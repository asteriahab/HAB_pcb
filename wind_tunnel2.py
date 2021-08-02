#same as wind_tunnel.py with some modfications
#include pressure, temperature, and imu code
#code to start running as soon as pi is switched on

import smbus
import time
import board
import busio
import adafruit_adt7410
import csv

# Reads num_bytes of data from data_addr in sens_addr and converts to int
# Assumes MSB is first
def readi2c(bus, sens_addr, data_addr, num_bytes):
    return int.from_bytes(bus.read_i2c_block_data(sens_addr, data_addr, num_bytes), "big")

# Define the address of the pressure sensor and read all 6 calibration variables
# Returns the sensor address and calibration data
def pres_init(bus):
    pres_addr = 0x76 # MS5803_02BA address

    # 6 calibration values addresses are 2 bytes each starting at 0xA2
    cal_addr = 0xA2
    cal_data_addrs = [cal_addr+2*n for n in range(6)]
    cal_data = [readi2c(bus, pres_addr, addr, 2) for addr in cal_data_addrs]

    return [pres_addr, cal_data]

# Define the address of the temperature sensor and configures the sensor
# Returns Adafruit object for the sensor
def temp_init():
    temp_addr = 0x48 #ADT7410 address
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    adt = adafruit_adt7410.ADT7410(i2c_bus, address=temp_addr)
    adt.high_resolution = True
    return adt

# Define the address of the accelerometer and configure the sensor
# Returns the sensor address
def accel_init(bus):
    dev_addr = 0x69 #ICM-20984 address

    # define useful registers
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    INT_ENABLE = 0x1B
    PWR_MGMT_1 = 0X6B

    # write to sample rate register
    bus.write_byte_data(dev_addr, SMPLRT_DIV, 7)    #check number 7
    # write to power management register
    bus.write_byte_data(dev_addr, PWR_MGMT_1, 1)
    # write to config register
    bus.write_byte_data(dev_addr, CONFIG, 0)
    # write to configure gyro register
    bus.write_byte_data(dev_addr, GYRO_CONFIG, 24)
    # write to interrupt enable register
    bus.write_byte_data(dev_addr, INT_ENABLE, 1)

    return dev_addr

# Reads and returns the temperature from the temperature sensor
def read_temp(adt):
    return adt.temperature

# Read, process, and return accelerometer data from addr
def read_acc_data(bus, dev_addr, addr):
    # not sure where these calculations come from
    return 9.81 * readi2c(bus, dev_addr, addr, 2)/16384.0

# Read and return three-axis of accelermoter data
def read_accel(bus, dev_addr):
    acc_x = read_acc_data(bus, dev_addr, 0x3b)
    acc_y = read_acc_data(bus, dev_addr, 0x3d)
    acc_z = read_acc_data(bus, dev_addr, 0x3f)

    return acc_x, acc_y, acc_z

# Read and return the digital pressure and temperature values
# Calculate and return the temperature difference and actual temperature
def read_pres_setup(bus, pres_addr, tempRef, tempCoefRef):
    bus.write_byte(pres_addr, 0x40) #pressure conversion cmd
    time.sleep(0.5)

    dpres = readi2c(bus, pres_addr, 0x00, 3)

    bus.write_byte(pres_addr, 0x50) # temp conversion cmd
    time.sleep(0.5)

    dtemp = readi2c(bus, pres_addr, 0x00, 3)

    # using formulas from datasheet:
    dT = dtemp - tempRef * pow(2,8) #diff between actual and ref temps
    TEMP = 2000 + dT * tempCoefRef / pow(2,23) # actual temperature

    return dpres, dtemp, dT, TEMP

# Read, process, and return pressure data from pressure sensor
def read_pres(bus, pres_addr, cal_data):
    dpres, dtemp, dT, TEMP = read_pres_setup(bus, pres_addr, cal_data[4], cal_data[5])

    # calculate offset and sensitivity using formula from datasheet
    OFF  = cal_data[1]  * pow(2,17) + (cal_data[3]  * dT)  / pow(2,6)
    SENS = cal_data[0] * pow(2,16) + (cal_data[2] * dT ) / pow(2,7)

    # for temps above 20C
    T2 = 0
    OFF2 = 0
    SENS2 = 0

    # second order temperature compensation is required for temperatures below 20C for higher accuracy
    # calculations taken from datasheet
    if TEMP < 2000:
        T2 = pow(dT,2) / pow(2,31)
        OFF2 = 61 * pow((TEMP - 2000),2) / pow(2,4)
        SENS2 = 2 * pow((TEMP - 2000),2)

    if TEMP < -1500:
        OFF2  = OFF2 + 15 * pow((TEMP + 1500),2)
        SENS2 = SENS2 + 8 * pow((TEMP + 1500),2)

    # adjust temperature, offset, and sensitivity readings for low temperatures
    TEMP -= T2
    OFF -= OFF2
    SENS -= SENS2

    # calculate final pressure reading
    pressure = (dpres * SENS / pow(2,21) - OFF) / pow(2,15) / 100.0

    return pressure

def main():
    bus = smbus.SMBus(1)

    # initialize all sensors
    pres_addr,pres_cal_data = pres_init(bus)
    accel_addr = accel_init(bus)
    adt = temp_init()

    # Open file where data will be collected
    with open('wind_tunnel.csv', mode='w') as datafile:
        # initialize file writer and add header row to csv
        write_data = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        write_data.writerow(['Time Elapsed', 'Temp', 'Pressure', 'X', 'Y', 'Z'])

        # record current time for reference
        t0 = time.time()

        # keep reading data for given number of seconds
        while (time.time()-t0) < 50:
            # read temperature, pressure, and accelerometer data
            temp = read_temp(adt)
            pres = read_pres(bus, pres_addr, pres_cal_data)
            acc_x, acc_y, acc_z = read_accel(bus, accel_addr)

            # add collected data to csv
            write_data.writerow([time.time()-t0, temp, pres, acc_x, acc_y, acc_z])

            #delay to prevent excessive data collection
            time.sleep(0.5)

main()
