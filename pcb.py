import smbus
import time
import board
import busio
import adafruit_adt7410
import qwiic_icm20948
import csv

# Used to initialize the sensors and bus
bus = None
IMU = None
adt = None
pres_cal_data = None

PRES_ADDR = 0x76 # MS5803_02BA address
TEMP_ADDR = 0x48 # ADT7410 address
IMU_ADDR  = 0x69 # ICM20948 address
PRES_CAL_ADDR = 0xA2 # Starting address for pressure sensor calibration data
PRES_CONVERT_CMD = 0x40 # Conversion command to read digital pressure value on pressure sensor
TEMP_CONVERT_CMD = 0x50 # Conversion command to read digital temperature value on pressure sensor

# Read num_bytes of data from data_addr in sens_addr and converts to int
def read_i2c(sens_addr, data_addr, num_bytes):
    global bus

    try:
        return int.from_bytes(bus.read_i2c_block_data(sens_addr, data_addr, num_bytes), "big")
    except:
        bus_init()

    return None

# Write given value for the given address to the i2c bus
def write_i2c(addr,val):
    global bus
    
    try:
        bus.write_byte(addr, val)
        return True
    except:
        bus_init()

    return False

# Initialize the i2c bus
def bus_init():
    global bus

    try:
        bus = smbus.SMBus(1)
    except:
        bus = None

# Read all 6 2-byte calibration variables for the pressure sensor
def pres_init():
    global pres_cal_data

    pres_cal_data = [read_i2c(PRES_ADDR, addr, 2) for addr in range (PRES_CAL_ADDR, PRES_CAL_ADDR+11, 2)]
    if None in pres_cal_data:
        pres_cal_data = None

# Initialize the temperature sensor object
def temp_init():
    global adt

    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        adt = adafruit_adt7410.ADT7410(i2c_bus, address=TEMP_ADDR)
        adt.high_resolution = True
    except:
        adt = None

# Initialize the IMU object
def imu_init():
    global IMU

    try:
       IMU = qwiic_icm20948.QwiicIcm20948(IMU_ADDR)
       if IMU.connected:
           IMU.begin()
    except:
        IMU = None

# Read all 9-axis inertial measurements from the IMU's accelerometer, gyroscope, and magnetorquer
def read_imu():
    global IMU

    try:
        if IMU.dataReady():
            IMU.getAgmt()
            return [IMU.axRaw, IMU.ayRaw, IMU.azRaw, IMU.gxRaw, IMU.gyRaw, IMU.gzRaw, IMU.mxRaw, IMU.myRaw, IMU.mzRaw]
    except:
        imu_init()

    return [None for _ in range(9)]

# Read the temperature from the temperature sensor
def read_temp():
    global adt

    try:
        return adt.temperature
    except:
        temp_init()

    return None

def calc_pres_vars(dtemp):
    global pres_cal_data
    
    # using formulas from datasheet:
    dT = dtemp - pres_cal_data[4] * pow(2,8) #diff between actual and ref temps
    TEMP = 2000 + dT * pres_cal_data[5] / pow(2,23) # actual temperature

    # calculate offset and sensitivity using formula from datasheet
    OFF  = pres_cal_data[1]  * pow(2,17) + (pres_cal_data[3]  * dT)  / pow(2,6)
    SENS = pres_cal_data[0] * pow(2,16) + (pres_cal_data[2] * dT ) / pow(2,7)

    return [dT, TEMP, OFF, SENS]

# Apply second order temperature compensation for temperatures below 20C for higher accuracy
def add_pres_comp(dT, TEMP, OFF, SENS):
    # calculations taken from datasheet
    v1 = pow((TEMP - 2000),2)
    v2 = pow((TEMP + 1500),2)
    
    if TEMP < 2000:              
        OFF -= 61 * v1 / pow(2,4)
        SENS -= 2 * v1

    if TEMP < -1500:            
        OFF -= 15 * v2
        SENS -= 8 * v2
        
    TEMP -= pow(dT,2) / pow(2,31) if TEMP < 2000 else 0

    return [TEMP, OFF, SENS]

# Reads raw digital data from the pressure sensor with a conversion command
def read_pres_digital(cmd):
    if write_i2c(PRES_ADDR, cmd):
        time.sleep(0.5)
        return read_i2c(PRES_ADDR, 0x00, 3)
    else:
        return None

# Read and process pressure data from pressure sensor
def read_pres():
    global pres_cal_data

    # make sure calibration data was collected successfully
    if pres_cal_data == None or not all(isinstance(x,int) for x in pres_cal_data):
        pres_init()
        return None

    # read digital pressure and temperature value
    dpres = read_pres_digital(PRES_CONVERT_CMD)
    dtemp = read_pres_digital(TEMP_CONVERT_CMD)

    # make sure digital values were collected successfully
    if dpres == None or dtemp == None:
        pres_init()
        return None

    dT, TEMP, OFF, SENS = calc_pres_vars(dtemp)
    # compensate calculations for low temperatures
    TEMP, OFF, SENS = add_pres_comp(dT, TEMP, OFF, SENS)

    # calculate final pressure reading
    pressure = (dpres * SENS / pow(2,21) - OFF) / pow(2,15) / 100.0

    return pressure

def main():
    bus_init()
    pres_init()
    imu_init()
    temp_init()

    with open('/home/pi/Asteria_HAB/data/{}.csv'.format(round(time.time())), mode='a') as datafile:
        filewriter = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        filewriter.writerow(['Time','Temp','Pressure','aX','aY','aZ','gX','gY','gZ','mX','mY','mZ'])

        while True:
            temp = read_temp()
            pres = read_pres()
            ax,ay,az,gx,gy,gz,mx,my,mz = read_imu()

            filewriter.writerow([time.time(), temp, pres, ax, ay , az, gx, gy, gz, mx, my, mz])
            datafile.flush() #make sure no data left in file buffer

            time.sleep(0.5)

if __name__ == "__main__":
    main()

# For testing:
def clear_bus():
    global bus
    bus = None

def get_bus():
    global bus
    return bus

def clear_adt():
    global adt
    adt = None

def get_adt():
    global adt
    return adt

def clear_imu():
    global IMU
    IMU = None

def get_imu():
    global IMU
    return IMU

def clear_pres_cal_data():
    global pres_cal_data
    pres_cal_data = None

def get_pres_cal_data():
    global pres_cal_data
    return pres_cal_data
