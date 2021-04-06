from __future__ import print_function
from typing import Any
import qwiic
import os
import time
import tsys01
import qwiic_icm20948
import sys
from time import sleep
from datetime import datetime
import board
import busio
import adafruit_gps
import serial
from adafruit_lsm6ds import LSM6DS33
import RPi.GPIO as GPIO
import glob
import signal
import math
import numpy as np
import rockblock_tools

print("DAQ Test Initiated")

# Function for creating new files - Returns String
def log_setup():
	#filename = imput("entire desired filename: ") # use if user input is desired
	filename = "BuoyDAQ_Data"
	file_path = "~/" + filename + ".csv" # saves in the root directory
	temp_file = open(file_path, "a") # opens file object for appending data
	if os.stat(file_path).st_size == 0:
		usr_input = ["Date(YYYYMMDD)","Time(hhmmss)","Lat","Lon","BattV","AT(C)","WT(C)","WVH(m)","DPD(s)","MWD(deg)","MXWVH(m)"]
		temp_file.write(usr_input+"\n") # writes headers to file and then newline to wait for data
	time.sleep(5)
	temp_file,close()
	return file_path

# Function for saving Data String to specified file - Returns void
def new_log(file_path, file_data="No Data Entered"):
	temp_file = open(file_path, "a")
	temp_file.write(file_data)
	temp_file.flush()
	time.sleep(5)
	temp_file.close()

# Function for sending data via RockBLOCK 9603 - Returns void
def sat_transmit(data):  # data : string of data to be sent via satellite
	# Arguments for send funtion : (IMEI, Username, Password, data)
	# IMEI is linked to RockBLOCK ... Can be found on Rock7 site
	rockblock_tools.send("300434064604330", "cshill@d.umn.edu", "BuoyDAQ_Drift", data)


# Function for obtaining gps data - returns array
def gps_data():
	gpsData = np.empty([1,8])
	# GPS Data is gathered
	if not gps.has_fix: # No GPS Signal detected
		for x in range(8):
			gpsData[0,x] = "NaN"
	else: # GPS Signal detected
		gpsData[0,0] = '{}'.format(gps.timestamp_utc.tm_year)
		gpsData[0,1] = '{}'.format(gps.timestamp_utc.tm_mon)
		gpsData[0,2] = '{}'.format(gps.timestamp_utc.tm_mday)
		gpsData[0,3] = '{:02}'.format(gps.timestamp_utc.tm_hour)
		gpsData[0,4] = '{:02}'.format(gps.timestamp_utc.tm_min)
		gpsData[0,5] = '{:02}'.format(gps.timestamp_utc.tm_sec)
		gpsData[0,6] = '{0:.5f}'.format(gps.latitude) # .5 gives 3ft accuracy, .6 gives 3inch accuracy
		gpsData[0,7] = '{0:.5f}'.format(gps.longitude)
	return gpsData


# MAIN

IMU_start = 1200 # Start IMU cycle at 20 min
IMU_end = 2400 # End IMU cycle at 40 min
IMU_Hz = 600 # 4 Hz - IMU sample rate
SAT_Hz = 600  # 10 minutes - SAT
AT_Hz = 1  # Default to 60 seconds - Air Temp
WT_Hz = 1 # Default to 60 seconds - Water Temp
ATsum = 0  # Used in calculating average air temp
WTsum = 0  # Used in calculating average water temp
ATavg = 0  # Average air temp since last sat transmission
WTavg = 0  # Average water temp since last sat transmission
ATsamp = 0  # Air temp samples taken since last sat trans
WTsamp = 0  # Water temp samples taken since last sat trans

# Disable GPIO warnings (these can be safely ignored in this case)
GPIO.setwarnings(False)

infLoop = True  # Bool to exit while-loop

# Use Pi UART (Tx/Rx) for GPS
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)

# Create a GPS module instance
gps = adafruit_gps.GPS(uart, debug=False) # Use UART/pyserial

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Changing 2000ms will adjust sampling rate 1/2000ms = 2Hz
gps.send_command(b'PMTK220,2000')

# Use Pi UART (Tx/Rx) for Sat Comm (9603)
ser = serial.Serial("/dev/ttyUSB0", 19200, 8, 'N', 1, timeout=10)

# Create variable for Temperature sensors
ATdata = np.array([["Temp(C)","Year","Month","Day","Hour","Min","Sec","Lat","Long"],["--","--","--","--","--","--","--","--","--"]])	# initialize an array for storing AT data
WTdata = np.array([["Temp(C)","Year","Month","Day","Hour","Min","Sec","Lat","Long"],["--","--","--","--","--","--","--","--","--"]])	# initialize an array for storing WT data
# initialize an array for storing IMU data
IMUdata = np.array([["Date","Time","AccelX","AccelY","AccelZ","GyroX","GyroY","GyroZ","MagX","MagY","MagZ","Temp"],["--","--","--","--","--","--","--","--","--","--","--","--"]])

# Initialize the 8-channel Qwiic Multiplexer
mux = qwiic.QwiicTCA9548A()

# Create an IMU object
IMU = qwiic_icm20948.QwiicIcm20948()
mux.disable_channels([0,1,2,3,4,5,6,7])
mux.enable_channels([0])
IMU.begin()

# Main loop runs forever printing the location, etc. every second.
last_log = time.monotonic()
imu_log = time.monotonic()
h2o_time = time.monotonic()
air_time = time.monotonic()
imu_cycle = time.monotonic()
start_time = time.monotonic()

while infLoop:
	# Checks for new GPS data
	gps.update()

	# Counters of time passed
	current = time.monotonic()
	secondary = time.monotonic()
	tempa_check = time.monotonic()
	tempw_check = time.monotonic()
	imu_check = time.monotonic()
	imu_cyclecheck = time.monotonic()
	onehour = time.monotonic()

	# if one hour has passed, reset IMU cycle counter
	if onehour - start_time >= 3600:
		start_time = onehour
		imu_cycle = onehour

# --------------------------
# AIR TEMPERATURE SENSOR
# --------------------------
	if tempa_check - air_time >= AT_Hz:
		air_time = tempa_check
		#time.sleep(0.1)
		print("recording air temp")
		ATnow = np.array([])
		mux.disable_channels([0,1,2,3,4,5,6,7])
		mux.enable_channels([1])
		AT = tsys01.TSYS01()  # Make new object for air temp sensor
		AT.init()  # Initialize air temp sensor
		#time.sleep(0.1)
		if not AT.read(): # AT sensor can't be read
			ATnow = "NaN"
		else: # AT sensor is readable
			ATnow = "%.2f" % AT.temperature(tsys01.UNITS_Centigrade)
			temp = AT.temperature(tsys01.UNITS_Centigrade)
			ATsum = ATsum + temp
			ATsamp = ATsamp + 1
		GPSdata = gps_data()  # obtains gps data size array
		ATnow = np.append(ATnow, GPSdata)
		ATdata = np.append(ATdata, [ATnow], axis = 0)
		mux.disable_channels([1])
		#print(ATdata) # Displays AT data in real-time
		np.savetxt('AT_data.csv', ATdata, fmt='%s', delimiter = ',')

# ----------------------------
# WATER TEMPERATURE SENSOR
# ----------------------------
	if tempw_check - h2o_time >= WT_Hz:
		h2o_time = tempw_check
		#time.sleep(0.1)
		print("recording water temp")
		WTnow = np.array([])
		mux.disable_channels([0,1,2,3,4,5,6,7])
		mux.enable_channels([3])
		WT = tsys01.TSYS01()  # Make new object for water temp sensor
		WT.init()  # Initialize water temp sensor
		#time.sleep(0.1)
		if not WT.read(): # WT sensor can't be read
			WTdata = "NaN"
		else: # WT sensor is readable
			WTnow = "%.2f" % WT.temperature(tsys01.UNITS_Centigrade)
			temp = WT.temperature(tsys01.UNITS_Centigrade)
			WTsum = WTsum + temp
			WTsamp = WTsamp + 1
		GPSdata = gps_data() # obtains gps data array
		WTnow = np.append(WTnow, GPSdata)
		WTdata = np.append(WTdata, [WTnow], axis = 0)
		#time.sleep(0.1)
		mux.disable_channels([3])
		#print(WTdata) # Displays WT data in real-time
		np.savetxt('WT_data.csv', WTdata, fmt='%s', delimiter = ',')

# --------------------------------
# IMU SENSOR
# --------------------------------
	# Checks if it is time to begin IMU cycle
	if (imu_cyclecheck - imu_cycle >= IMU_start) and (imu_cyclecheck - imu_cycle <= IMU_end):
		# Checks if it is time to gather IMU data/log it
		if imu_check - imu_log >= IMU_Hz:
			# print("gathering IMU data")
			imu_log = imu_check
			#time.sleep(0.1)
			# Take reading from IMU on Mux channel 0
			mux.disable_channels([0,1,2,3,4,5,6,7])
			mux.enable_channels([0])
			#time.sleep(0.1)
			if IMU.connected == False:
				print("The IMU is not connected")

			#IMU.begin()           #PROBLEMS WITH THIS FUNCTION, MESSES UP CYCLE

			IMUnow = np.empty([1,12])  # stores current IMU data

			# Accel (G force +/-): float(value) / scale_accel. - scale_accel takes on 16.384 (for 2g, default)

			# Gyro (degrees per second): float(value) / scale_gyr. -scale_gyr takes on 131.072 (for 500dps, default)

			# Mag: float(value) * 0.15 - 3-axis magnetic sensor with full-scale range of +/- 4900 micro-T

			if IMU.dataReady():
				IMU.getAgmt()  # read all axis and temp from sensor
						# note this also updates all instance variables
				IMUnow[0,0] = datetime.utcnow().strftime("%Y%m%d")
				IMUnow[0,1] = datetime.utcnow().strftime("%H%M%S.%f")[:-4]
				IMUnow[0,2] = '{: 06d}'.format(IMU.axRaw)
				IMUnow[0,3] = '{: 06d}'.format(IMU.ayRaw)
				IMUnow[0,4] = '{: 06d}'.format(IMU.azRaw)
				IMUnow[0,5] = '{: 06d}'.format(IMU.gxRaw)
				IMUnow[0,6] = '{: 06d}'.format(IMU.gyRaw)
				IMUnow[0,7] = '{: 06d}'.format(IMU.gzRaw)
				IMUnow[0,8] = '{: 06d}'.format(IMU.mxRaw)
				IMUnow[0,9] = '{: 06d}'.format(IMU.myRaw)
				IMUnow[0,10] = '{: 06d}'.format(IMU.mzRaw)
				IMUnow[0,11] = '{: 06d}'.format(IMU.tmpRaw)

				#IMUnow[0,0] = datetime.utcnow().strftime("%Y%m%d")
				#IMUnow[0,1] = datetime.utcnow().strftime("%H%M%S.%f")[:-4]
				#IMUnow[0,2] = float(IMU.axRaw) / 16.384
				#IMUnow[0,3] = float(IMU.ayRaw) / 16.384
				#IMUnow[0,4] = float(IMU.azRaw) / 16.384
				#IMUnow[0,5] = float(IMU.gxRaw) / 131.072
				#IMUnow[0,6] = float(IMU.gyRaw) / 131.072
				#IMUnow[0,7] = float(IMU.gzRaw) / 131.072
				#IMUnow[0,8] = float(IMU.mxRaw) * 0.15
				#IMUnow[0,9] = float(IMU.myRaw) * 0.15
				#IMUnow[0,10] = float(IMU.mzRaw) * 0.15
				#IMUnow[0,11] = (float(IMU.tmpRaw) / 333.87) + 21

			IMUdata = np.append(IMUdata, IMUnow, axis = 0)
			#time.sleep(0.1)
			mux.disable_channels([0])
			np.savetxt('IMU.csv', IMUdata, fmt = '%s', delimiter=',')

#---------------------------
# SATELLITE TRANSMISSION
# --------------------------
	if current - last_log >= SAT_Hz:
		last_log = current

		# GPS Data is gathered
		if not gps.has_fix: # No GPS Signal detected
			lat = "NaN"
			long = lat
			theDate = datetime.now().date()
			satData = theDate + str(lat)
			satellite_transmit(satData)
		else:
			# GPS Signal detected
			satData = '{}{}{}{:02}{:02}{:02},'.format(
				gps.timestamp_utc.tm_year,
				gps.timestamp_utc.tm_mon,
				gps.timestamp_utc.tm_mday,
				gps.timestamp_utc.tm_hour,
				gps.timestamp_utc.tm_min,
				gps.timestamp_utc.tm_sec)
			lat = '{0:.5f}'.format(gps.latitude) # .5 gives 3ft accuracy, .6 gives 3inch accuracy
			long = '{0:.5f}'.format(gps.longitude)

		# Calculating averages
		if ATsamp == 0:
			ATavg = "NaN"
		else:
			ATavg = ATsum/ATsamp
			ATavg = round(ATavg,2)
		if WTsamp == 0:
			WTavg = "NaN"
		else:
			WTavg = WTsum/WTsamp
			WTavg = round(WTavg,2)

		# Transmits Data
		satData = satData + str(lat) + ',' + str(long) + ',' + str(ATavg) + ',' + str(WTavg)
		print("transmitting data")
		sat_transmit(satData)

		# Reset sums and sample sizes
		ATsum = 0
		WTsum = 0
		ATsamp = 0
		WTsamp = 0
