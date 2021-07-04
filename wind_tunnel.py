#include pressure, temperature code
#code to start running as soon as pi is switched on
#added acceleration

#used libraries
import smbus
import time
import board
import busio
import adafruit_adt7410
import csv

def pres_init():
    bus = smbus.SMBus(1)

def temp_init():
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    adt = adafruit_adt7410.ADT7410(i2c_bus, address=0x48)
    adt.high_resolution = True
    return adt

def accel_init():
    #NEED TO ADD REGISTERS
    # write to sample rate register
    bus.write_byte_data(0x69, 0x19, 7)    #check number 7
    # write to power management register
    bus.write_byte_data(0x69, PWR_MGMT_1, 1)
    # write to config register
    bus.write_byte_data(0x69, CONFIG, 0)
    # write to configure gyro register
    bus.write_byte_data(0x69, GYRO_CONFIG, 24)
    # write to interrupt enable register
    bus.write_byte_data(0x69, INT_ENABLE, 1)

def read_temp(adt):
    temp = adt.temperature()
    return temp

def read_accel():
        high = bus.read_byte_data(0x69, reg_addr)
        low = bus.read_byte_data(0x69, reg_addr+1)

        value = (high << 8) | low

        # get signed value
        if(value > 32768):
            value -= 65536
        return value

def read_pres():
    bus.write_byte(0x76, 0x1E)
    time.sleep(0.5)
    # Read 12 bytes of calibration data
    # Read pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA2, 2)  #0xA2 =
    C1 = data[0] * 256 + data[1]
    # Read pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA4, 2)
    C2 = data[0] * 256 + data[1]
    # Read temperature coefficient of pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA6, 2)
    C3 = data[0] * 256 + data[1]
    # Read temperature coefficient of pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA8, 2)
    C4 = data[0] * 256 + data[1]
    # Read reference temperature
    data = bus.read_i2c_block_data(0x76, 0xAA, 2)
    C5 = data[0] * 256 + data[1]
    # Read temperature coefficient of the temperature
    data = bus.read_i2c_block_data(0x76, 0xAC, 2)
    C6 = data[0] * 256 + data[1]
    # MS5803_02BA address, 0x76(118)
    #		0x40(64)	Pressure conversion(OSR = 256) command
    bus.write_byte(0x76, 0x40)
    time.sleep(0.5)
    # Read digital pressure value
    # Read data back from 0x00(0), 3 bytes
    # D1 MSB2, D1 MSB1, D1 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D1 = value[0] * 65536 + value[1] * 256 + value[2]
    # MS5803_02BA address, 0x76(118)
    #		0x50(64)	Temperature conversion(OSR = 256) command
    bus.write_byte(0x76, 0x50)
    time.sleep(0.5)
    # Read digital temperature value
    # Read data back from 0x00(0), 3 bytes
    # D2 MSB2, D2 MSB1, D2 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D2 = value[0] * 65536 + value[1] * 256 + value[2]

    dT = D2 - C5 * 256
    TEMP = 2000 + dT * C6 / 8388608
    OFF = C2 * 131072 + (C4 * dT) / 64
    SENS = C1 * 65536 + (C3 * dT ) / 128
    T2 = 0
    OFF2 = 0
    SENS2 = 0

    if TEMP >= 2000 :
    	T2 = 0
    	OFF2 = 0
    	SENS2 = 0
    elif TEMP < 2000 :
    	T2 = (dT * dT) / 2147483648
    	OFF2= 61 * ((TEMP - 2000) * (TEMP - 2000)) / 16
    	SENS2= 2 * ((TEMP - 2000) * (TEMP - 2000))
    	if TEMP < -1500 :
    		OFF2 = OFF2 + 20 * ((TEMP + 1500) * (TEMP + 1500))
    		SENS2 = SENS2 + 12 * ((TEMP + 1500) * (TEMP +1500))

    TEMP = TEMP - T2
    OFF = OFF - OFF2
    SENS = SENS - SENS2
    pressure = ((((D1 * SENS) / 2097152) - OFF) / 32768.0) / 100.0
    cTemp = TEMP / 100.0

    return pressure, cTemp

def main():
    pres_init()
    adt = temp_init()

    with open('wind_tunnel.csv', mode='w') as datafile:
        write_data = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        t0 = time.time()

        # keep reading data when elapse time is less than 1 hour
        while (t0 - time.time()) < 3600:
            temp = read_temp(adt)
            pres, temp_ref = read_pres()
            acc_x = read_accel()
            acc_y = read_accel()
            acc_z = read_accel()
            temp_writer.writerow([temp, pres, temp_ref, acc_x, acc_y, acc_z, time.time()])
            #delay to allow for data to be collected
            sleep(0.5)


main()
