#same as wind_tunnel.py with some modfications
#include pressure, temperature, and imu code
#code to start running as soon as pi is switched on

import smbus
import time
import board
import busio
import adafruit_adt7410
import csv

def pres_init(bus):
    pres_addr = 0x76    # MS5803_02BA address, 0x76(118)
    bus.write_byte(pres_addr, 0x1E) # reset
    time.sleep(0.5)

    # calibration data addresses:
    # [pres sens, pres off, temp coef sens, temp coef off, ref temp, temp coef of temp]
    cal_data_addrs = [0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC]
    cal_data = [int.from_bytes(bus.read_i2c_block_data(pres_addr, addr, 2)) for addr in cal_data_addrs]

    return cal_data

def temp_init():
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    adt = adafruit_adt7410.ADT7410(i2c_bus, address=0x48)
    adt.high_resolution = True
    return adt

def accel_init():
    # id of mpu sensor (found through i2cdetect command)
    dev_addr = 0x69
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

def read_temp(adt):
    temp = adt.temperature()
    return temp

def read_acc_data(bus, dev_addr, addr):
    #not sure where these calculations come from
    return 9.81 * int.from_bytes(bus.ready_i2c_block_data(dev_addr, addr, 2), "big")/16384.0

def read_accel(bus, dev_addr):
    acc_x = read_acc_data(bus, dev_addr, 0x3b)
    acc_y = read_acc_data(bus, dev_addr, 0x3d)
    acc_z = read_acc_data(bus, dev_addr, 0x3f)

    return acc_x, acc_y, acc_z

#read the digital pressure and temperature values
#calculate the temperature difference and actual temperature
def read_pres_setup(bus, tempRef, tempCoefRef):
    bus.write_byte(pres_addr, 0x1E) # reset

    dpres = int.from_bytes(bus.read_i2c_block_data(pres_addr, 0x00, 3), "big")

    bus.write_byte(pres_addr, 0x50) #temp conversion cmd
    time.sleep(0.5)

    dtemp = int.from_btes(bus.read_i2c_block_data(pres_addr, 0x00, 3), "big")

    #using formulas from datasheet:
    dT = dtemp - tempRef * pow(2,8) #diff between actual and ref temps
    TEMP = 2000 + dT * tempCoefRef / pow(2,23) #actual temperature

    return dpres, dtemp, dT, TEMP

#Read Pressure Value
def read_pres(bus, cal_data):
    dpres, dtemp, dT, TEMP = read_pres_setup(bus, cal_data[4], cal_data[5])

    #calculate offset and sensitivity using formula from datasheet
    OFF  = cal_data[1]  * pow(2,17) + (cal_data[3]  * dT)  / pow(2,6)
    SENS = cal_data[0] * pow(2,16) + (cal_data[2] * dT ) / pow(2,7)

    # for temps above 20C
    T2 = 0
    OFF2 = 0
    SENS2 = 0

    #second order temperature compensation is required for temperatures below 20C for higher accuracy
    #calculations taken from datasheet
    if TEMP < 2000:
        T2 = pow(dT,2) / pow(2,31)
        OFF2 = 61 * pow((TEMP - 2000),2) / pow(2,4)
        SENS2 = 2 * pow((TEMP - 2000),2)

    if TEMP < -1500:
        OFF2  = OFF2 + 15 * pow((TEMP + 1500),2)
        SENS2 = SENS2 + 8 * pow((TEMP + 1500),2)

    #adjust temperature, offset, and sensitivity readings for low temperatures
    TEMP -= T2
    OFF -= OFF2
    SENS -= SENS2

    pressure = (dpres * SENS / pow(2,21) - OFF) / pow(2,15) / 100.0
    cTemp = TEMP / 100.0

    return pressure, cTemp

def main():
    bus = smbus.SMBus(1)

    pres_cal_data = pres_init()
    accel_addr = accel_init()
    adt = temp_init()

    with open('wind_tunnel.csv', mode='w') as datafile:
        write_data = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        t0 = time.time()

        # keep reading data when elapse time is less than 1 hour
        while (t0 - time.time()) < 3600:
            temp = read_temp(adt)
            pres, temp_ref = read_pres(bus, pres_cal_data)
            acc_x, acc_y, acc_z = read_accel(bus, accel_addr)
            temp_writer.writerow([temp, pres, temp_ref, acc_x, acc_y, acc_z, time.time()])
            #delay to allow for data to be collected
            sleep(0.5)

main()
