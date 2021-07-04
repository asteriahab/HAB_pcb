#useful libraries
import time
import board
import busio
import adafruit_adt7410
import csv

#code template from: https://circuitpython.readthedocs.io/projects/adt7410/en/latest/

#initialise the i2c bus
i2c_bus = busio.I2C(board.SCL, board.SDA)
adt = adafruit_adt7410.ADT7410(i2c_bus, address=0x48)
adt.high_resolution = True

#defining registers and default settings
#PLEASE CHECK
reg_temp = 0x00
reg_config = 0x03
crit_Th = 64 #default
crit_Tl = 10 #default

#defining start time
t0 = time.time()

with  open('temp_Vals.csv', mode='w') as temp_data:
    temp_writer = csv.writer(temp_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    t0 = time.time()
    # keep reading data when elapse time is less than 2 hours
    while (t0 - time.time()) < 7200:
        #returns temperature in celsius
        temp = adt.temperature()
        temp_writer.writerow([temp, time.time()])
        #delay to allow for data to be collected
        sleep(1)
