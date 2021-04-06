from typing import Any
from __future__ import print_function
import time
import qwiic
import qwiic_icm20948
import sys
import os
from time import sleep
from datetime import datetime
import board
import busio
import adafruit_gps
import serial
from adafruit_lsm6ds import LSM6DS33
import RPi.GPIO as GPIO
import tsys01
import glob
import signal
import math
import numpy as np

#import adafruit_ads1x15.ads1015 as ADS # Uncomment to add voltage monitoring (12-Bit ADC)
#import adafruit_ads1x15.ads1115 as ADS # Uncomment to add voltage monitoring (16-bit ADC)
#import adafruit_ads1x15.analog_in import AnalogIn # Uncomment when either ADC is added

# Function for creating new files - Returns String
def log_setup():
    # filename = input("Enter desired filename: ") # use if user input desired
    filename = "BuoyDAQ_Data"
    file_path = "~/" + filename + ".csv" # saves in the root directory
    # print("File will be saved as" + file_path) # print commands unnecessary for remote logging w/ no UI
    temp_file = open(file_path, "a") # opens file object for appending data
    if os.stat(file_path).st_size == 0:
        # print("Give desired column names and separate with ','\ni.e. Time,Date,Lat,Long")
        # usr_input = input("Column Names: ")
        usr_input = ["Date(YYYYMMDD)","Time(hhmmss)","Lat","Lon","BattV","AT(C)","WT(C)",...
                     "WVH(m)","DPD(s)","MWD(deg)","MXWVH(m)"]
        temp_file.write(usr_input+"\n") # writes headers to file and then newline to wait for data
    time.sleep(5)
    temp_file.close()
    return file_path

# Function for saving Data String to specified file - Returns void
def new_log(file_path, file_data="No Data Entered"):
    temp_file = open(file_path, "a")
    temp_file.write(file_data)
    temp_file.flush()
    time.sleep(5)
    temp_file.close()

# Function that Handles ASCII transmission via RockBLOCK 9603
def satellite_transmit(message_out):
    againtry = True
    set_try_time = time.monotonic()
    while againtry:
        cur_try_time = time.monotonic() 
        print(len(message_out), "If less then 50 should cost only 1 credit")
        # Setup communication to Sat Comm Module
        #ser = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, timeout=10)
        global ser
        ser.write(b"AT+CSQ\r")  # Command to check signal strength
        response = ser.read(5) # Debugging to check antenna
        print(response.decode())
        print("*******************************************")
        ser.write(b"AT\r")
        response = ser.read(4)
        print("*******************************************")
        ser.write(b"AT&K0") # Disable Flow Control
        response = ser.read(11)
        print(response.decode(), "&K0 Called")
        print("+++++++++++++++++++++++++++++++++++++++++++")
        ser.write(("AT+SBDWT=" + message_out + "\r").encode())  # Sends ASCII Message
        formatted_msg = "AT+SBDWT=" + message_out + "\r"
        ser.write(formatted_msg.encode())
        response = ser.read(20)
        print(response.decode(), "+SBDWT Called")
        print("___________________________________________")
        ser.write(b"AT+SBDIX\r")  # Checks if message was sent
        response = ser.read(66)
        valuech = response.decode()
        print(valuech)
        if "SBDIX:" in valuech:
            print("Command recieved")
            againtry = False
        else:
            print("Issue with final Command, retrying...")
        if cur_try_time - set_try_time >= 240:
            againtry = False
    ser.close()

# Function that allows users to adjust sampling rates with no need for coding knowledge - Returns void
def sample_setup():
    global GPS_Hz
    global AT_Hz
    global WT_Hz
    global SAT_Hz
    global IMU_Hz_start
    global IMU_Hz_end
    print('#' * 40)
    print('#' * 40)
    print("Default sample rates are as follow: \nGps:1min\nSatellite:10min\nAir Temp:10sec\nWater Temp:10sec")
    print("IMU samples for 20min between 20-40min mark\n")
    print('#' * 40)
    print("Enter the number zero if you want to use default sample rates.")
    print('#' * 40)
    while True:
        try:
            input_gpsrate = int(input("Enter GPS sample rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
            sys.exit()
    if input_gpsrate > 0:
        GPS_Hz = input_gpsrate*60 # not really Hz, but instead seconds per sample
    print('#' * 40)
    while True:
        try:
            input_satrate = int(input("Enter Satellite Transmission Rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_satrate > 0:
        SAT_Hz = input_satrate*60 # not really Hz, but instead seconds per sample
    print('#' * 40)
    while True:
        try:
            input_airtemprate = int(input("Enter Air Temperature sample rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_airtemprate > 0:
        AT_Hz = input_airtemprate*60 # not really Hz, but instead seconds per sample
    print('#' * 40)
    while True:
        try:
            input_watertemprate = int(input("Enter Water Temperature sample rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_watertemprate > 0:
        WT_Hz = input_watertemprate*60 # not really Hz, but instead seconds per sample
    print('#' * 40)
    print("Now the IMU will be setup. \nFirst a start time will be given.")
    print("i.e. If 30 is given. IMU will begin recording at the 30th minute of the hour.")
    while True:
        try:
            input_imustarttime = int(input("Enter the START time of IMU data recording (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_imustarttime > 0:
        IMU_Hz_start = input_imustarttime*60
    print('#' * 40)
    print("Now the IMU end time will be given.")
    print("If 40 is given. IMU will end recording data around the 40th minute of the hour.")
    print("Example: If 30 was given as the start time and 40 is given for the end time.")
    print("Sampling will take place between HH:30 and HH:40.")
    print("Meaning there is a total sampling time of 10min")
    while True:
        try:
            input_imuendtime = int(input("Enter the END time of the IMU data recording (in minutes): "))
            if IMU_Hz_start/60 < input_imuendtime:
                break
            else:
                print("End time must be greater then the start time!")
        except ValueError:
            print("Must enter an integer value!")
    if input_imuendtime > 0:
        IMU_Hz_end = input_imuendtime*60
    print('#' * 40)
    print('#' * 40)
    print("New Sample Rates are as follow:\nGps:" + str(int(GPS_Hz/60)) + "min\nSatellite:" +       str(int(SAT_Hz/60)) + "min\nAir Temp:" + str(int(AT_Hz/60)) + "min") 
    print("Water Temp:"+str(int(WT_Hz/60)) + "min\nIMU samples for " + str(int((IMU_Hz_end-IMU_Hz_start)/60)) + "min between " + str(int(IMU_Hz_start/60)) + "-" + str(int(IMU_Hz_end/60)) + "min mark.")
    print('#' * 40)


# =============================================================================
# =============================================================================
# =============================================================================
# End Setup Functions     
# =============================================================================
# =============================================================================
# =============================================================================
IMU_Hz_start = 1200  # Start 20min after the hour
IMU_Hz_end = 2400  # End at 40min after the hour
GPS_Hz = 60  # Default to 1 minute for GPS
SAT_Hz = 600  # Default to 10 minutes for Sat Comm
AT_Hz = 10  # Default to 10 seconds (6 samples per minute) - Air Temp
WT_Hz = 10  # Default to 10 seconds (6 samples per minute) - Water Temp
adc_Rate = 1800 # Default to 30 minutes for ADC

# Disable GPIO warnings (these can be safely ignored in this case)
GPIO.setwarnings(False)

# Allows User to Adjust Sample rates
#sample_setup() # Comment out to disallow user given sample rates

# Bool to exit while-loop
infLoop = True
# ***
# ***
# =============================================================================
# Use Pi UART (Tx/Rx) for GPS
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Changing 2000ms will adjust sampling rate 1/2000ms = 2Hz 
gps.send_command(b'PMTK220,2000')
# =============================================================================
# ***
# ***
# =============================================================================
# Use Pi UART (Tx/Rx) for Sat Comm (9603)
ser = serial.Serial("/dev/ttyUSB0", baudrate=19200, 8, 'N', 1, timeout=10)
# =============================================================================
# ***
# ***
# =============================================================================
# Create variable for Temperature sensors
ATdata = np.array([]) # initialize an array for storing AT data
WTdata = np.array([]) # initialize an array for storing WT data
ATdata = "NaN"
WTdata = "NaN"
# =============================================================================
# ***
# ***
# =============================================================================
# Create an ADC I2C connection
#sensor_ADC = ADS.ADS1015(i2c) # Uncomment if using the ADS1015 ADC
#sensor_ADC = ADS.ADS1115(i2c) # Uncomment if using the ADS1115 ADC
# =============================================================================
# ***
# ***
# =============================================================================
# Initialize the 8-channel Qwiic Multiplexer for IMU, AT, WT, etc.
MUX8 = qwiic.QwiicTCA9548A()

# Create and IMU object
IMU = qwiic_icm20948.QwiicIcm20948()

# =============================================================================
# Sets file up for logging Sensors (not IMU)
# -- Latitude, Longitude -> degrees
# -- Temperatures -> Degrees Centigrade
# -- Time -> 24hr clock (needs to be in UTC!!!)
# -- Date -> YYYYMMDD
# =============================================================================
data_File = open("~/BuoyDAQ_Data.csv", "a") # need to append date and time to filename
if os.stat("~/BuoyDAQ_Data.csv").st_size == 0: # checks to see if file already exists
    StatsHdr_input = ["Date(YYYYMMDD)","Time(hhmmss)(UTC)","Lat","Lon","BattV","AT(C)","WT(C)",...
                     "WVH(m)","DPD(s)","MWD(deg)","MXWVH(m)"]
    data_File.write(StatsHdr_input + "\n")
time.sleep(5)
data_File.close()
# =============================================================================
# ***
# ***
# =============================================================================
# Setup of IMU file
# Accel = Acceleration -> m/s^2
# Gyro  = Gyroscope -> degree/s
imu_File = open("~/IMU_Data.csv", "a") # need to append date and time to filename
if os.stat("~/IMU_Data.csv").st_size == 0: # checks to see if file already exists
    IMUhdr_input = ["Date(YYYYMMDD)","Time(hhmmss)","Lat","Lon","BattV","AT(C)","WT(C)",...
                     "WVH(m)","DPD(s)","MWD(deg)","MXWVH(m)"]
    imu_File.write(IMUhdr_input + "\n")
time.sleep(5)
imu_File.close()
# =============================================================================
# ***
# ***
# ***
# =============================================================================
# Main loop runs forever printing the location, etc. every second.
last_log = time.monotonic()
imu_log = time.monotonic()
h2o_time = time.monotonic()
air_time = time.monotonic()
#adc_time = time.monotonic() # Uncomment if you want to add seperate timer for ADC
# =============================================================================
# ***
# ***
# ***
# =============================================================================
while infLoop:

    # Checks for new GPS data
    gps.update()

    # Counters of time passed.
    current = time.monotonic()
    secondary = time.monotonic()
    tempa_check = time.monotonic()
    tempw_check = time.monotonic()
    timeOday = datetime.now().time()

# -------------------------------
# AIR TEMPERATURE SENSOR
# -------------------------------
    # NEED TO ADD DATE AND TIME TO EACH AT AND WT MEASUREMENT
    # SAVE AT AND WT DATA TO SEPARATE FILE JUST LIKE IMU DATA
    # REPORT BACK SUMMARY STATS AS AVERAGE OF LAST XXX MINUTES, JUST LIKE WAVES
    if tempa_check - air_time >= AT_Hz:
        air_time = tempa_check
        time.sleep(0.1)
        MUX8.disable_channels([0,1,2,3,4,5,6,7])
        MUX8.enable_channels([1])
        # List Channel Configuration
        # MUX8.list_channels() # will show a list of enabled and disabled channels
        # Enable air temp on MUX8 channel 1
        AT = tsys01.TSYS01()  # Make new object for air temp sensor
        AT.init()  # Initialize air temp sensor
        time.sleep(0.1)
        if not AT.read():
            ATdata = np.append(ATdata,nan)
            # ATdata = "NaN"
        else:
            ATnow = "%.2f" % AT.temperature(tsys01.UNITS_Centigrade)
            ATdata = np.append(ATdata,ATnow)
            time.sleep(0.1)
        MUX8.disable_channels([1])
        # np.savetxt('AT_Data.csv', ATdata, delimiter=',')
      
# -------------------------------
# WATER TEMPERATURE SENSOR
# -------------------------------
    # NEED TO ADD DATE AND TIME TO EACH AT AND WT MEASUREMENT
    # SAVE AT AND WT DATA TO SEPARATE FILE JUST LIKE IMU DATA
    # REPORT BACK SUMMARY STATS AS AVERAGE OF LAST XXX MINUTES, JUST LIKE WAVES
    if tempw_check - h2o_time >= WT_Hz:
        h2o_time = tempw_check
        time.sleep(0.1)
        MUX8.disable_channels([0,1,2,3,4,5,6,7])
        MUX8.enable_channels([2])
        # List Channel Configuration
        # MUX8.list_channels() # will show a list of enabled and disabled channels
        # Enable water temp on MUX8 channel 2
        WT = tsys01.TSYS01()  # Make new object for air temp sensor
        WT.init()  # Initialize air temp sensor
        time.sleep(0.1)
        if not WT.read():
            WTdata = "NaN"
        else:
            WTdata = "%.2f" % WT.temperature(tsys01.UNITS_Centigrade)
            sleep(0.1)
        MUX8.disable_channels([2])
        # np.savetxt('WT_Data.csv', WTdata, delimiter=',')
        
# =============================================================================
        #   NEED TO FIGURE OUT HOW TO ACCUMULATE DATA ARRAY, PROCESS STATS, WRITE ARRAY TO FILE, THEN DUMP AFTER SAVING AND SENDING TO START FRESH.
#         # Log Date, Time, and Temperatures
#         data_File = open("~/BuoyDAQ_Data.csv", "a")
#         data_File.write(str(theDate)+","+str(timeOday)+","+lat+","+long+"," ...
#                         + WTdata + "," + ATdata + "\n")
#         data_File.flush()
#         time.sleep(5)
#         data_File.close()
# =============================================================================
           
# -------------------------------
# GPS DATA
# -------------------------------
    # Checks if it is time to gather GPS data/Log it
    if current - last_log >= GPS_Hz:
        last_log = current
        
        # GPS Data is gathered
        if not gps.has_fix: # No GPS Signal detected
            lat = "NaN"
            long = lat
            # timeOday = datetime.now().time()
            theDate = datetime.now().date()
            satData = theDate + str(lat)
            satellite_transmit(satData)
        else:
            # GPS Signal detected
            timeOday = '{:02}{:02}{:02}'.format(
                gps.timestamp_utc.tm_hour,
                gps.timestamp_utc.tm_min,
                gps.timestamp_utc.tm_sec)
            theDate = '{}{}{}'.format(
                gps.timestamp_utc.tm_year,
                gps.timestamp_utc.tm_mon,
                gps.timestamp_utc.tm_mday)
            satData = '{}{}{}{:02}{:02}{:02},'.format(
                gps.timestamp_utc.tm_year,
                gps.timestamp_utc.tm_mon,
                gps.timestamp_utc.tm_mday,
                gps.timestamp_utc.tm_hour,
                gps.timestamp_utc.tm_min,
                gps.timestamp_utc.tm_sec)
            lat = '{0:.5f}'.format(gps.latitude) # .5 gives 3ft accuracy, .6 gives 3inch accuracy
            long = '{0:.5f}'.format(gps.longitude)
            
            # Transmits Data if GPS signal present
            satData = satData + str(lat) + ',' + str(long) + ','
            #print(satData, "Data to be transmitted!") # Uncomment for debugging Sat Comm
            satellite_transmit(satData)
            
        # Log Date, Time, GPS data, and Temperatures
        data_File = open("~/BuoyDAQ_Data.csv", "a")
        data_File.write(str(theDate)+","+str(timeOday)+","+lat+","+long+"," ...
                        + WTdata + "," + ATdata + "\n")
        data_File.flush()
        time.sleep(5)
        data_File.close()

# -------------------------------
# IMU SENSOR
# -------------------------------
        # collect IMU data at 10 hz for 20 minutes (= 10*60*20 = 12000 samples)
        # Checks if it is time to gather IMU data/Log it
        if secondary - imu_log >= IMU_Hz_start:
            
            # initialize imu storage array for 20 minutes at 10 hz
            imu_temp = np.empty([12000,12])
            
            # Take reading from IMU on Mux Channel 0
            MUX8.disable_channels([0,1,2,3,4,5,6,7])
            MUX8.enable_channels([0])
            # List Channel Configuration
            # MUX8.list_channels() # will show a list of enabled and disabled channels

            imu_working = True
            imu_data_set = " "
            while (imu_working)  # Continue reading IMU data until
                if secondary - imu_log >= IMU_Hz_end:
                    imu_log = secondary
                    imu_working = False
                    
                # IMU Data is gathered between 20-40 minutes after the hour by default
# =============================================================================
#                 acc = "{: 06d}, %{: 06d}, %{: 06d}" % sensor_IMU.acceleration
#                 gyr = "{: 06d}, %{: 06d}, %{: 06d}" % sensor_IMU.gyro
#                 mag = "{: 06d}, %{: 06d}, %{: 06d}" % sensor_IMU.gyro
# =============================================================================
                
                 #   NEED TO FIGURE OUT HOW TO ACCUMULATE DATA ARRAY, PROCESS STATS, WRITE ARRAY TO FILE, THEN DUMP AFTER SAVING AND SENDING TO START FRESH.
                 
                if IMU.dataReady():
            		
                    IMU.getAgmt() # read all axis and temp from sensor
                        # note this also updates all       instance variables    
                    imu_File = open("~/IMU_Data.csv", "a")
                    imu_File.write(str(theDate)+","+str(timeOday)+","...
                                   +'{: 06d}'.format(IMU.axRaw)+...
                                   +'{: 06d}'.format(IMU.ayRaw)+...
                                   +'{: 06d}'.format(IMU.azRaw)+...
                                   +'{: 06d}'.format(IMU.gxRaw)+...
                                   +'{: 06d}'.format(IMU.gyRaw)+...
                                   +'{: 06d}'.format(IMU.gzRaw)+...
                                   +'{: 06d}'.format(IMU.mxRaw)+...
                                   +'{: 06d}'.format(IMU.myRaw)+...
                                   +'{: 06d}'.format(IMU.mzRaw)+...
                                   +'{: 06d}'.format(IMU.tmpRaw)+"\n")
                    imu_File.flush()
                    #time.sleep(2)
                    imu_File.close()
                    
