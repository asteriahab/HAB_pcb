import smbus
import time
import board
import busio
import adafruit_adt7410
import qwiic_icm20948
import csv

bus = None #Initialized in main
IMU = None # //
adt = None # //
pres_cal_data = None #

pres_addr = 0x76 # MS5803_02BA address
temp_addr = 0x48 #ADT7410 address
imu_addr  = 0x69 #ICM20948 address

pres_cal_addr = 0xA2 #starting address for pressure sensor calibration data

# Reads num_bytes of data from data_addr in sens_addr and converts to int
# Assumes MSB is first
def readi2c(sens_addr, data_addr, num_bytes):
    if bus:
        try:
            return int.from_bytes(bus.read_i2c_block_data(sens_addr, data_addr, num_bytes), "big")
        except:
            pass
    else:
        init_bus()

    return None

def writei2c(addr,val):
    if bus:
        try:
            bus.write_byte(addr, val)
            return True
        except:
            pass
    else:
        init_bus()

    return False

def bus_init():
    try:
        bus = smbus.SMBus(1)
    except:
        pass
# Define the address of the pressure sensor and read all 6 calibration variables
# Returns the sensor address and calibration data
def pres_init():
    # 6 calibration values addresses are 2 bytes each starting at 0xA2
    cal_data_addrs = [pres_cal_addr+2*n for n in range(6)]
    pres_cal_data = [readi2c(pres_addr, addr, 2) for addr in cal_data_addrs]
    if None in pres_cal_data:
        pres_cal_data = None

# Define the address of the temperature sensor and configures the sensor
# Returns Adafruit object for the sensor
def temp_init():
    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        adt = adafruit_adt7410.ADT7410(i2c_bus, address=temp_addr)
        adt.high_resolution = True
    except:
        return None

# Initializes qwiic_icm20948 object accessing the imu
def imu_init():
    try:
       IMU = qwiic_icm20948.QwiicIcm20948(imu_addr)
       if IMU.connected:
           IMU.begin()
    except:
        pass

# Reads all 9-axis inertial measurements from the IMU's accelerometer, gyroscope, and magnetorquer
def read_imu(IMU):
    if IMU:
        try:
            if IMU.dataReady():
                IMU.getAgmt()
                return [IMU.axRaw, IMU.ayRaw, IMU.azRaw, IMU.gxRaw, IMU.gyRaw, IMU.gzRaw, IMU.mxRaw, IMU.myRaw, IMU.mzRaw]
        except:
            pass
    else:
        imu_init()

    return [None for _ in range(9)]

# Reads and returns the temperature from the temperature sensor
def read_temp():
    if adt:
        try:
            return adt.temperature
        except:
            pass
    else:
        temp_init()

    return None

# Apply second order temperature compensation for temperatures below 20C for higher accuracy
def add_pres_comp(dT, TEMP, OFF, SENS):
    T2 = 0
    OFF2 = 0
    SENS2 = 0

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

    return [TEMP, OFF, SENS]

# Read, process, and return pressure data from pressure sensor
def read_pres():
    if not pres_cal_data:
        pres_init()
        return None
    # read digital pressure value
    if writei2c(pres_addr, 0x40): #pressure conversion cmd
        time.sleep(0.5)
        dpres = readi2c(pres_addr, 0x00, 3)
    else:
        return None

    # read digital temperature value
    if writei2c(pres_addr, 0x50): # temp conversion cmd
        time.sleep(0.5)
        dtemp = readi2c(pres_addr, 0x00, 3)
    else:
        return None

    # using formulas from datasheet:
    dT = dtemp - pres_cal_data[4] * pow(2,8) #diff between actual and ref temps
    TEMP = 2000 + dT * pres_cal_data[5] / pow(2,23) # actual temperature

    # calculate offset and sensitivity using formula from datasheet
    OFF  = pres_cal_data[1]  * pow(2,17) + (pres_cal_data[3]  * dT)  / pow(2,6)
    SENS = pres_cal_data[0] * pow(2,16) + (pres_cal_data[2] * dT ) / pow(2,7)

    TEMP, OFF, SENS = add_pres_comp(dT, TEMP, OFF, SENS)

    # calculate final pressure reading
    pressure = (dpres * SENS / pow(2,21) - OFF) / pow(2,15) / 100.0

    return pressure

def main():
    # initialize all sensors and i2c bus
    init_bus()
    pres_init()
    imu_init()
    temp_init()

    # Open file where data will be collected
    with open('wind_tunnel.csv', mode='w') as datafile:
        # initialize file writer and add header row to csv
        write_data = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        write_data.writerow(['Time Elapsed', 'Temp', 'Pressure', 'aX', 'aY', 'aZ', 'gX', 'gY', 'gZ', 'mX', 'mY', 'mZ'])

        # record current time for reference
        t0 = time.time()

        # keep reading data for given number of seconds
        while (time.time()-t0) < 60:
            # read temperature, pressure, and accelerometer data
            temp = read_temp(adt)
            pres = read_pres(pres_cal_data)
            ax,ay,az,gx,gy,gz,mx,my,mz = read_imu(IMU)

            # add collected data to csv
            write_data.writerow([time.time()-t0, temp, pres, ax, ay, az, gx, gy, gz, mx, my, mz])

            #delay to prevent excessive data collection
            time.sleep(0.5)

main()
