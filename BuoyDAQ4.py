from __future__ import print_function
from typing import Any
import qwiic
import os
import time
import tsys01
import qwiic_icm20948
import sys
from time import sleep
import datetime
import board
import busio
import adafruit_gps
import serial
from adafruit_lsm6ds import LSM6DS33
import RPi.GPIO as GPIO
import glob
import signal
import math as m
import cmath as cm
import numpy as np
import struct
from adafruit_rockblock import RockBlock
from numpy.fft import fft, fftfreq
import serial
from scipy.signal import filtfilt, butter
print("DAQ Test Initiated")

# Function for sending data via RockBLOCK 9603 - Returns void
def sat_transmit(data):  # data : string of data to be sent via satellite
    ser = serial.Serial("/dev/ttyUSB0", 19200)
    rb = RockBlock(ser)
    #bindata = struct.pack("i", len(data)) # converts the data to binary
    rb.text_out = data  # put data in outbound buffer
    print("Talking to satellite...")
    status = rb.satellite_transfer()
    # loop as needed
    retry = 0
    while status[0] > 8 and retry < 8:  # Keeps trying until data is sent. Max retries = 5
        time.sleep(10)
        status = rb.satellite_transfer()
        print(retry, status)
        retry += 1
    if status[0] > 8:
        print("Transmission failed")
    print("\nDONE.")

# Function for obtaining gps data - returns array
def gps_data():
    gpsData = np.empty([1,4])  # Initialize empty array for GPS data
    # GPS Data is gathered
    if not gps.has_fix: # No GPS Signal detected
        for x in range(3):
            gpsData[0,x] = "NaN"
    else: # GPS Signal detected
        gpsData[0,0] = '{}'.format(gps.timestamp_utc.tm_year) + '{}'.format(gps.timestamp_utc.tm_mon) +  '{}'.format(gps.timestamp_utc.tm_mday)
        gpsData[0,1] = '{:02}'.format(gps.timestamp_utc.tm_hour) + '{:02}'.format(gps.timestamp_utc.tm_min) + '{:02}'.format(gps.timestamp_utc.tm_sec)
        gpsData[0,2] = '{0:.5f}'.format(gps.latitude) # .5 gives 3ft accuracy, .6 gives 3inch accuracy
        gpsData[0,3] = '{0:.5f}'.format(gps.longitude)
    return gpsData


# MAIN

IMU_start = 1200 # Start IMU cycle at 20 min
IMU_end = 2400 # End IMU cycle at 40 min
IMU_Hz = .25 # 4 Hz - IMU sample rate
IMU_samp = int((IMU_end - IMU_start)/IMU_Hz) # Total number of IMU samples to be obtained
SAT_Hz = 600  # Default to 10 minutes - SAT
TEMP_Hz = 20  # Default to 60 seconds - Temp
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

# initiliaze an array for storing AT data
TEMPdata = np.array([["AirTemp(C)","WaterTemp(C)","YYYYMMDD","hhmmss","Lat","Long"],["--","--","--","--","--","--"]])

# Initialize the 8-channel Qwiic Multiplexer
mux = qwiic.QwiicTCA9548A()

# Create an IMU object
IMU = qwiic_icm20948.QwiicIcm20948()
mux.disable_channels([0,1,2,3,4,5,6,7])
mux.enable_channels([0])
IMU.begin()

# Timer variables
last_log = time.perf_counter()
imu_log = time.perf_counter()
temp_time = time.perf_counter()
imu_cycle = time.perf_counter()
start_time = time.perf_counter()

IMUdata = np.empty([IMU_samp,12])  # Array for storing IMU data
IMU_n = 0  # Current number of IMU samples obtained

# Temperature file name,
AWTfilename = datetime.datetime.utcnow().strftime("%Y%m%d")

postdone = False

# ---------------------------
# Main Loop
# ---------------------------
# Runs on an infinite loop, gathering and sending data at the specified rate

while infLoop:
    # Checks for new GPS data
    gps.update()

    # Timer variables
    current = time.perf_counter()
    secondary = time.perf_counter()
    temp_check = time.perf_counter()
    imu_check = round(time.perf_counter(), 2)
    imu_cyclecheck = time.perf_counter()
    onehour = time.perf_counter()

    # if one hour has passed, reset IMU cycle counter
    if onehour - start_time >= 3600:
        start_time = onehour
        imu_cycle = onehour
        IMU_n = 0  # reset the number of IMU samples to 0

# -----------
# IMU SENSOR
# -----------

    # Checks if it is time to begin IMU cycle
    if (imu_cyclecheck - imu_cycle >= IMU_start) and (IMU_n <= IMU_samp):
        # Checks if it is time to gather IMU data/log it
        if imu_check - imu_log >= IMU_Hz:
            imu_log = imu_check
            # If this is first sample, name the file with current date and time
            if IMU_n == 0:
                filename = datetime.datetime.utcnow().strftime("%Y%m%d") + "_" +  datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-6]
            # Take reading from IMU on Mux channel 0
            mux.disable_channels([0,1,2,3,4,5,6,7])
            mux.enable_channels([0])
            if IMU.connected == False:
                print("The IMU is not connected")
            if (IMU_n < IMU_samp):  # Collect another sample
                if IMU.dataReady():
                    IMU.getAgmt()  # read all axis and temp from sensor
                                   # note this also updates all instance variables

                    IMUdata[IMU_n,0] = datetime.datetime.utcnow().strftime("%Y%m%d")
                    IMUdata[IMU_n,1] = datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-4]

                    # IMU Post-Processing will utilize this raw data
                    IMUdata[IMU_n,2] = float(IMU.axRaw)
                    IMUdata[IMU_n,3] = float(IMU.ayRaw)
                    IMUdata[IMU_n,4] = float(IMU.azRaw)
                    IMUdata[IMU_n,5] = float(IMU.gxRaw)
                    IMUdata[IMU_n,6] = float(IMU.gyRaw)
                    IMUdata[IMU_n,7] = float(IMU.gzRaw)
                    IMUdata[IMU_n,8] = float(IMU.mxRaw)
                    IMUdata[IMU_n,9] = float(IMU.myRaw)
                    IMUdata[IMU_n,10] = float(IMU.mzRaw)
                    IMUdata[IMU_n,11] = float(IMU.tmpRaw)

                    IMU_n = IMU_n + 1  # Samples obtained increases by 1

            if (IMU_n >= IMU_samp):  # All IMU data has been gathered, save data to file
                print("saving IMU data")
                np.savetxt(filename + 'IMU' + '.csv', IMUdata, fmt='%s', delimiter=',')
                IMU_n = IMU_n + 1

            mux.disable_channels([0])

# --------------------
# TEMPERATURE SENSORS
# --------------------

    # Checks if it is time to collect temperature data
    if temp_check - temp_time >= TEMP_Hz:
        temp_time = temp_check
        TEMPnow = np.array([])
        mux.disable_channels([0,1,2,3,4,5,6,7])

    # Air Temp
        mux.enable_channels([1])
        AT = tsys01.TSYS01()  # Make new object for air temp sensor
        try:  # Try block will keep program running if data can not be read from sensor
            AT.init()  # Initialize air temp sensor
            if not AT.read(): # AT sensor can't be read
                ATnow = "NaN"
            else: # AT sensor is readable
                ATnow = "%.2f" % AT.temperature(tsys01.UNITS_Centigrade)
                temp = AT.temperature(tsys01.UNITS_Centigrade)
                ATsum = ATsum + temp
                ATsamp = ATsamp + 1
        except IOError:
            ATnow = "NaN"  # If remote I/O Error (i.e. sensor not connected)
        mux.disable_channels([1])

    # Water Temp
        mux.enable_channels([3])
        WT = tsys01.TSYS01()  # Make new object for water temp sensor
        try:  # These try blocks should be implemented for all sensors (GPS, Sat) to prevent unwanted program terminations
            WT.init()  # Initialize water temp sensor
            if not WT.read(): # WT sensor can't be read
                WTdata = "NaN"
            else: # WT sensor is readable
                WT.init()
                WTnow = "%.2f" % WT.temperature(tsys01.UNITS_Centigrade)
                temp = WT.temperature(tsys01.UNITS_Centigrade)
                WTsum = WTsum + temp
                WTsamp = WTsamp + 1
        except IOError:
            WTnow = "NaN"
        mux.disable_channels([3])

        GPSdata = gps_data()  # obtains gps data size array
        TEMPnow = np.append(TEMPnow, ATnow) # Append temperature array with Air temp
        TEMPnow = np.append(TEMPnow, WTnow) # Append temperature array with Water temp
        TEMPnow = np.append(TEMPnow, GPSdata) # Append temperature array with GPS data
        TEMPdata = np.append(TEMPdata, [TEMPnow], axis = 0)
        # Save temperature data to file
        np.savetxt(AWTfilename + 'AWT.csv', TEMPdata, fmt='%s', delimiter = ',')


# --------------------------
# SATELLITE TRANSMISSION
# --------------------------

    # Checks if it is time to transmit data. Does not allow for transmission during IMU cycle.
    # Transmissions can take a while and delay IMU cycle. This should be improved upon.
    if current - last_log >= SAT_Hz and not((imu_cyclecheck - imu_cycle >= IMU_start) and (IMU_n <= IMU_samp)):
        last_log = current
        time.sleep(0.01)
        # GPS Data is gathered
        if not gps.has_fix: # No GPS Signal detected
            lat = "NaN"
            long = "NaN"
            theDate = datetime.datetime.utcnow().date()
            satData = str(theDate) + str(lat)
            sat_transmit(satData)
        else: # GPS Signal detected
            satData = '{}/{}/{}, {:02}:{:02}:{:02}, '.format(
            gps.timestamp_utc.tm_year,
            gps.timestamp_utc.tm_mon,
            gps.timestamp_utc.tm_mday,
            gps.timestamp_utc.tm_hour,
            gps.timestamp_utc.tm_min,
            gps.timestamp_utc.tm_sec)
            lat = '{0:.5f}'.format(gps.latitude) # .5 gives 3ft accuracy, .6 gives 3inch accuracy
            long = '{0:.5f}'.format(gps.longitude)

        # Calculating averages
        if ATsamp == 0:  # if no air samples have been taken since last transmission
            ATavg = "NaN"
        else:
            ATavg = ATsum/ATsamp  # calculates average air temp since last transmission
            ATavg = round(ATavg,2) # round to 2 decimal places
        if WTsamp == 0:  # if no waters samples have been taken since last transmission
            WTavg = "NaN"
        else:
            WTavg = WTsum/WTsamp  # calculates average water temp since last transmission
            WTavg = round(WTavg,2) # round to 2 decimal places

        # Transmits Data
        # Data transmitted: Date, time, latitude, longitude, average air temp, average water temp
        satData = satData + str(lat) + ', ' + str(long) + ', ' + str(ATavg) + ', ' + str(WTavg)
        print("data to be transmitted = ")
        print(satData)
        sat_transmit(satData)

        # Reset sums and sample sizes
        ATsum = 0
        WTsum = 0
        ATsamp = 0
        WTsamp = 0

# --------------------
# IMU POST-PROCESSING
# --------------------

    if (IMU_n >= IMU_samp) and (postdone == False):  # All IMU data has been gathered and postprocessing has not been done this cycle
        g = 9.80665;                  # m/s^2, gravitational acceleration at Sea Level.
        b10 = 0.0                     # Assume no hull magnetic effects for Azimuth.
        b20 = 0.0                     # ^
        b11 = 1.0                     # ^
        b22 = 1.0                     # ^
        b12 = 0.0                     # ^
        b21 = 0.0                     # ^
        b, a = butter(4, 0.25)        # initial butterworth filter at 1 Hz.
        S2g = 16384                   # Least sensitive bit for +-2g. 
        S4g = 8192                    # Least sensitive bit for +-4g.
        S8g = 4096                    # Least sensitive bit for +-8g.
        S16g = 2048                   # Least sensitive bit for +-16g.
        # ================================================================================================================


        # ================================================================================================================
        # Name each column in the IMU csv file, order is important
        # name_list = ['date', 'time', 'axRaw', 'ayRaw', 'azRaw', 'gxRaw', 'gyRaw', 'gzRaw', 'mxRaw', 'myRaw', 'mzRaw', 'tmpRaw']

        # Read the csv and name each column accordingly
        # data = pd.read_csv(r'C:\Users\DMG12\.spyder-py3\IMU1meter.csv', names=name_list)
        # print (data.head())
        # ================================================================================================================


        # ================================================================================================================
        # Get the name_list arrays, the Raw data from each sensor.
        # Each sensor should be converted to proper units in csv script.
        # ================================================================================================================
        Time = IMUdata[:,1]                                    # sec
        Ax = IMUdata[:,2] / S8g                                # m/s^2
        Ay = IMUdata[:,3] / S8g                                # m/s^2
        Az = IMUdata[:,4] / S8g                                # m/s^2
        Gx = IMUdata[:,5] / 131.072                            # deg/sec
        Gy = IMUdata[:,6] / 131.072                            # deg/sec
        Gz = IMUdata[:,7] / 131.072                            # deg/sec
        Bx = IMUdata[:,8] * 0.15                               # mTesla
        By = IMUdata[:,9] * 0.15                               # mTesla
        Bz = IMUdata[:,10] * 0.15                              # mTesla
        tmp = (IMUdata[:,11] / 333.87) + 21.0                  # Celsius
        # ================================================================================================================


        # ================================================================================================================
        # Pull the length of the time array for integration steps.
        n = len(Time)                           # Set up the time array length to loop.
        print('Length of time array =', n)
        # ================================================================================================================


        # ================================================================================================================
        # Set the initialization arrays with length n. 
        # ================================================================================================================
        t = [0.0] * n                            # initial time array.
        Pitcha = [0.0] * n                       # initial accelerometer Pitch array.
        Rolla = [0.0] * n                        # initial accelerometer Roll array.
        norm = [0.0] * n                         # initial magnetometer normalization array.
        mx = [0.0] * n                           # initial magnetic effects on x axis.
        my = [0.0] * n                           # initial magnetic effects on y axis.
        mz = [0.0] * n                           # initial magnetic effects on z axis.
        Mx = [0.0] * n                           # initial corrected magnetic effects on x axis.
        My = [0.0] * n                           # initial corrected magnetic effects on y axis.
        Yawm = [0.0] * n                         # initial Yaw from magnetometer.
        dt = [0.0] * n                           # initial delta time.
        fs = [0.0] * n                           # initial sample frequency.
        Roll = [0.0] * n                         # initial roll from gyroscope.
        Pitch = [0.0] * n                        # initial pitch from gyroscope.
        Yaw = [0.0] * n                          # initial yaw from gyroscope.
        CFRoll = [0.0] * n                       # initial complimentary Roll filter.
        CFPitch = [0.0] * n                      # initial complimentary Pitch filter.
        CFYaw = [0.0] * n                        # initial complimentary Yaw filter.
        B1 = [0.0] * n                           # initial magnetic field in x axis.
        B2 = [0.0] * n                           # initial magnetic field in y axis.
        Bez = [0.0] * n                          # initial corrected magnetic effects on x axis.
        Bey = [0.0] * n                          # initial corrected magnetic effects on y axis.
        S = [0.0] * n                            # initial variable to compute magnetic Azimuth.
        C = [0.0] * n                            # ^
        D = [0.0] * n                            # ^
        Am = [0.0] * n                           # initial magnetic Azimuth.
        VAR = [-1.0778] * n                      # Calculate magnetic declination from gps lat and long.
        A = [0.0] * n                            # initial True North Azimuth, from Duluth House. 
        filt_A = [0.0] * n                       # initial filtered Azimuth.
        Zx = [0.0] * n                           # initial Zenith Deck East-West Slope.
        Zy = [0.0] * n                           # initial Zenith Deck North-South Slope.
        R = [0.0] * n                            # Direction Cosine vector.
        gx = [0.0] * n                           # initial x axis gravity component.
        gy = [0.0] * n                           # initial y axis gravity component.
        gz = [0.0] * n                           # initial z axis gravity component.
        Xs = [0.0] * n                           # initial x axis Buoy acceleration without gravity.
        Ys = [0.0] * n                           # initial y axis Buoy acceleration without gravity.
        Zs = [0.0] * n                           # initial z axis Buoy acceleration without gravity.
        a1 = [0.0] * n                           # initial matrix component to convert to Earth's reference frame. 
        b1 = [0.0] * n                           # ^ 
        c1 = [0.0] * n                           # ^
        a2 = [0.0] * n                           # ^
        b2 = [0.0] * n                           # ^
        c2 = [0.0] * n                           # ^ 
        a3 = [0.0] * n                           # ^
        b3 = [0.0] * n                           # ^
        c3 = [0.0] * n                           # ^
        Xe = [0.0] * n                           # initial x axis Acceleration in Earth's reference frame without gravity.
        Ye = [0.0] * n                           # initial y axis Acceleration in Earth's reference frame without gravity.
        Ze = [0.0] * n                           # initial z axis Acceleration in Earth's reference frame without gravity.
        Ex = [0.0] * n                           # initial x axis Acceleration in Earth's reference frame.
        Ey = [0.0] * n                           # initial y axis Acceleration in Earth's reference frame.
        Ez = [0.0] * n                           # initial z axis Acceleration in Earth's reference frame.
        Evx = [0.0] * n                          # initial x axis Velocity in Earth's reference frame.
        Evy = [0.0] * n                          # initial y axis Velocity in Earth's reference frame.
        Evz = [0.0] * n                          # initial z axis Velocity in Earth's reference frame.
        Epx = [0.0] * n                          # initial x axis Displacement in Earth's reference frame.
        Epy = [0.0] * n                          # initial y axis Displacement in Earth's reference frame.
        Epz = [0.0] * n                          # initial z axis Displacement in Earth's reference frame.
        freqband = [0.0] * n                     # initial frequency band.
        Qnv = [0.0] * n                          # initial quad spectra of North-South slope.
        Qwv = [0.0] * n                          # initial quad spectra of East-West slope.
        D = [0.0] * n                            # initial Wave Direction in radians.
        Dm = [0.0] * n                           # initial Wave Direction in degrees.
        dfn = [0.0] * n    # initial Hz, Bandwidth of nth band frequency. Source: https://www.ndbc.noaa.gov/wavecalc.shtml
        # ================================================================================================================


        # ================================================================================================================
        # Calculate the pitch and roll with the accelerometer, magnetometer and gyroscope.
        # Sensor Fusion technique will be used to allow for an accurate static and dynamic Buoy.
        # L. Tran Data Fusion with 9 Degrees of Freedom Inertial Measurement Unit To Determine 
        # Object’s Orientation along with Direction Cosine Law. Sources: http://www.starlino.com/imu_guide.html and
        # https://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?referer=https://www.google.com/&httpsredir=1&article=1422&context=eesp
        # ================================================================================================================
        for i in range(n):
        # Set the time array to start at zero.
            t[i] = Time[i] - Time[0]                                                 # Set time at zero in seconds.

            # if statement for second to minute deletion.
            if t[i] > t[i - 1] + 40:
                t[i] = t[i] - 40                                                     # Condition for continuous time.

        # ================================================================================================================
        # First Calculate the Pitch and Roll with accelerometer axis. 
        # Note this is only accurate for when the Buoy is Static. Source: http://www.starlino.com/imu_guide.html
        # Calculate the Pitcha.
        # ================================================================================================================
            R[i] = m.sqrt(Ax[i]**2 + Ay[i]**2 + Az[i]**2)                            # Acceleration vector normalization.
            Pitcha[i] = m.acos(Ax[i] / R[i])                                         # Radians.
            Pitcha[i] = m.degrees(Pitcha[i])                                         # Degrees.

            # Calculate the Rolla.
            Rolla[i] = m.acos(Ay[i] / R[i])                                          # Radians.
            Rolla[i] = m.degrees(Rolla[i])                                           # Degrees.
        # ================================================================================================================


        # ================================================================================================================
        # Calculate the Yaw from the magnetometer. L. Tran Data Fusion 2. Magnetometer.
        # ================================================================================================================
            norm[i] = m.sqrt(Bx[i]**2 + By[i]**2 + Bz[i]**2)                         # normalize each magnetic field axis.
            mx[i] = Bx[i] / norm[i]                                                  # uT/uT = unitless.
            my[i] = -1 * By[i] / norm[i]                                             # ^
            mz[i] = Bz[i] / norm[i]                                                  # ^
            Mx[i] = mx[i] * m.cos(Pitcha[i]) + mz[i] * m.sin(Pitcha[i])              # Radians.
            My[i] = mx[i] * m.sin(Rolla[i]) * m.sin(Pitcha[i]) + my[i] * m.cos(Rolla[i]) \
                - mz[i] * m.sin(Rolla[i]) * m.cos(Pitcha[i])                         # Radians.
            Yawm[i] = m.acos(Az[i] / R[i])                                           # Radians.
            Yawm[i] = m.degrees(Yawm[i])                                             # Degrees.
        # ================================================================================================================


        # ================================================================================================================
        # integrate Gx, Gy and Gz to get the pitch and role from gyroscope. Source: L. Tran
        # ================================================================================================================
            dt[i] = t[i] - t[i-1]                                                    # Delta time in seconds.
            if dt[i] == 0:
                dt[i] = 0.25                                                         # Reset to the sampling frequency.
            fs[i] = 1 / dt[i]                                                        # Sampling Frequency in Hz.

            # Set limit on delta time and do not allow negative time.
            if dt[i] > 1.0:
                dt[i] = 1.0
            else:
                if dt[i] < 0:
                    dt[i] = 0.01
            # integrate each gyroscope axis into Pitch, Role and Yaw 
            Roll[i] = Gx[i] * dt[i]                                                  # degrees.
            Pitch[i] = Gy[i] * dt[i]                                                 # degrees.
            Yaw[i] = Gz[i] * dt[i]                                                   # degrees.
        # ================================================================================================================


        # ================================================================================================================
        # Complimentary filter for Pitch, Roll and Yaw. Source: L. Tran
        # ================================================================================================================
            CFRoll[i] = 0.98 * (CFRoll[i] + Roll[i]) + 0.02 * Rolla[i]               # Degrees.
            CFPitch[i] = 0.98 * (CFPitch[i] + Pitch[i]) + 0.02 * Pitcha[i]           # Degrees.
            CFYaw[i] = 0.98 * (CFYaw[i] + Yaw[i]) + 0.02 * Yawm[i]                   # Degrees.
        # ================================================================================================================


        # ================================================================================================================
        # Calculate Azimuth. Source: Steel and Earle 1991, Eqs. 2-8.
        # Assume there are no hull magnetic effects (room for improvement here).
        # reassign magx and magy to B1 and B2.
        # ================================================================================================================
            B1[i] = mx[i]                                                            # Magnetic Field in X axis.
            B2[i] = my[i]                                                            # Magnetic Field in Y axis.
            Bez[i] = Mx[i]                                                           # Magnetic Field component in X axis.
            Bey[i] = My[i]                                                           # Magnetic Field component in Y axis.
            d = b11 * b22 - b12 * b21;                                               # Equation (7).
            S[i] = (b21 * m.cos(CFPitch[i]) + b22 * m.sin(CFPitch[i]) * m.sin(CFRoll[i])) * (B1[i] - b10) \
                - (b11 * m.cos(CFPitch[i]) - b12 * m.sin(CFPitch[i]) * m.sin(CFRoll[i])) * (B2[i] - b20) \
                - Bez[i] * d * m.sin(CFRoll[i])
            C[i] = (b22 * (B1[i] - b10) - b12 * (B2[i] - b20)) * m.cos(CFRoll[i]) - Bez[i] * d * \
                m.sin(CFPitch[i]) * m.cos(CFRoll[i])
            D[i] = Bey[i] * d * m.cos(CFPitch[i]) * m.cos(CFRoll[i])
            Am[i] = m.atan((S[i] / D[i]) / (C[i] / D[i]))                            # Equation (2), (3) and (8)
            Am[i] = m.degrees(Am[i])                                                 # Convert Magnetic Azimuth to degrees.
            A[i] = m.asin((m.sin(Am[i]) * m.cos(VAR[i]) + m.cos(Am[i]) * m.sin(VAR[i]))) # Calculate True Azimuth in Radians.
            A[i] = m.degrees(A[i])                                                   # Convert True Azimuth to degrees.
        # ================================================================================================================ 


        # ================================================================================================================ 
        # Zenith Deck East-West and Deck North-South slope calculations, Steel et al 1998 Pg. 125
        # ================================================================================================================
            Zx[i] = (m.sin(CFPitch[i]) / m.cos(CFPitch[i])) * m.sin(A[i]) - (m.sin(CFRoll[i]) \
                / (m.cos(CFRoll[i]) * m.cos(CFPitch[i]))) * m.cos(A[i])              # Deck East-West Slope Magnitude.
            Zy[i] = (m.sin(CFPitch[i]) / m.cos(CFPitch[i])) * m.cos(A[i]) - (m.sin(CFRoll[i]) \
                / (m.cos(CFRoll[i]) * m.cos(CFPitch[i]))) * m.sin(A[i])              # Deck North-South Slope Magnitude.
        # ================================================================================================================ 


        # ================================================================================================================
        # Plum Bob Acceleration Method with calculated Pitch and Roll. Source: http://www.starlino.com/imu_guide.html
        # ================================================================================================================
            gx[i] = Ax[i] / R[i]                                             # Gravity component in Buoy X axis.
            gy[i] = Ay[i] / R[i]                                             # Gravity component in Buoy Y axis.                                                      
            gz[i] = m.sqrt(1.0 - gx[i]**2 - gy[i]**2)                        # Gravity component in Buoy Z axis.
        # ================================================================================================================


        # ================================================================================================================
        # Correct for the +1g acceleration for calculations in x, y and z. 
        # Source: J. Atmos. Oceanic Technol. (2010) 27 (6): 1012–1028. Eq. (1)
        # https://journals.ametsoc.org/jtech/article/27/6/1012/3397/A-Comparison-of-Methods-for-Determining
        # ================================================================================================================
            Xs[i] = (-Ax[i] + gx[i])                                         # X axis Buoy Acceleration without gravity.
            Ys[i] = (-Ay[i] + gy[i])                                         # Y axis Buoy Acceleration without gravity.
            Zs[i] = (-Az[i] + gz[i])                                         # Z axis Buoy Acceleration without gravity. 
        # ================================================================================================================


        # ================================================================================================================
        # Correct to Earths Reference frame with these matrix components, Steel et al 1998 Pg. 125
        # Calculate the Heave from Method V Coefficients for Earths X, Y and Z positions from
        # J.Atmos. Oceanic Technol. (2010)
        # ================================================================================================================
            a1[i] = m.cos(CFPitch[i]) * m.cos(A[i])
            b1[i] = m.sin(CFRoll[i]) * m.sin(CFPitch[i]) * m.cos(A[i]) - m.cos(CFRoll[i]) * m.sin(A[i])
            c1[i] = m.cos(CFRoll[i]) * m.sin(CFPitch[i]) * m.cos(A[i]) - m.sin(CFRoll[i]) * m.sin(A[i])
            a2[i] = m.cos(CFPitch[i]) * m.sin(A[i])
            b2[i] = m.sin(CFRoll[i]) * m.sin(CFPitch[i]) * m.sin(A[i]) - m.cos(CFRoll[i]) * m.cos(A[i])
            c2[i] = m.cos(CFRoll[i]) * m.sin(CFPitch[i]) * m.sin(A[i]) - m.sin(CFRoll[i]) * m.cos(A[i])
            a3[i] = - m.sin(CFPitch[i])
            b3[i] = m.sin(A[i]) * m.cos(CFPitch[i])
            c3[i] = m.cos(A[i]) * m.cos(CFPitch[i])
        # ================================================================================================================


        # ================================================================================================================
        # Earth's reference frame from J.Atmos. Oceanic Technol. (2010)  (2).
        # ================================================================================================================
            Xe[i] = (a1[i] * Xs[i] + b1[i] * Ys[i] + c1[i] * Zs[i])          # Earth's X axis Acceleration without gravity.
            Ye[i] = (a2[i] * Xs[i] + b2[i] * Ys[i] + c2[i] * Zs[i])          # Earth's Y axis Acceleration without gravity.
            Ze[i] = (a3[i] * Xs[i] + b3[i] * Ys[i] + c3[i] * Zs[i])          # Earth's Z axis Acceleration without gravity.
        # ================================================================================================================


        # ================================================================================================================
        # Compute the Earth's reference frame x, y and z accelerations
        # True Earth's Heave reference frame acceleration from Method V Eq. (9) 
        # Source: https://journals.ametsoc.org/jtech/article/27/6/1012/3397/A-Comparison-of-Methods-for-Determining
        # ================================================================================================================
            Ex[i] = - g * Xe[i]                                                      # True Earth's X Acceleration in m/s^2.
            Ey[i] = - g * Ye[i]                                                      # True Earth's Y Acceleration in m/s^2.
            Ez[i] = g * (1.0 - Ze[i])                                                # True Earth's Heave Acceleration in m/s^2.
        # ================================================================================================================


        # ================================================================================================================
        # Compute the Earth's reference frame x, y and z velocities
        # True Earth's Heave reference frame velocity. Source: mk4, pg. (28) 7.3 Buoy axes and references
        # ================================================================================================================
            Evx[i] = Ex[i] * dt[i]                                                   # True Earth's X Velocity in m/s.
            Evy[i] = Ey[i] * dt[i]                                                   # True Earth's Y Velocity in m/s.
            Evz[i] = Ez[i] * dt[i]                                                   # True Earth's Heave Velocity in m/s.
        # ================================================================================================================


        # ================================================================================================================
        # Compute the Earth's reference frame x, y and z displacements
        # True Earth's Heave reference frame displacement. Source: mk4, pg. (28) 7.3 Buoy axes and references
        # ================================================================================================================
            Epx[i] = Evx[i] * dt[i]                                                  # True Earth's X Position in m.
            Epy[i] = Evy[i] * dt[i]                                                  # True Earth's Y Position in m.
            Epz[i] = Evz[i] * dt[i]                                                  # True Earth's Heave Position in m.
        # ================================================================================================================


        # ================================================================================================================
        # Fast Fourier Transform in Earth Reference frame and mask. 
        # Source: Teng 2002 - NDBC Buoys, Pg. 518
        # ================================================================================================================ 
        freqs = fftfreq(n, dt[i])                                               # Set up the FFT Nyquist frequency. 
        mask = freqs > 0.0                                                      # Mask the freqs for positive values.

        # ================================================================================================================
        # Calculate the FFT for the Heave and separate the complex conjugate. 
        # Source: https://www.ndbc.noaa.gov/wavecalc.shtml
        # ================================================================================================================
        fft_Epz = fft(Epz, n)                                                   # FFT for Heave in m/s^2.
        fft_Ep = (fft_Epz.real, fft_Epz.imag)                                   # Separate the real and imaginary numbers.
        Epr = fft_Epz.real                                                      # Real numbers.
        Epi = fft_Epz.imag                                                      # imaginary numbers.

        # ================================================================================================================
        # FFT for the Zenith Deck North-South and East-West Slopes. Source: mk4 pg. 30-31.
        # ================================================================================================================
        fft_Zy = fft(Zy, n)                                                     # FFT for North-South Deck Slope in rad/s.
        fftZy = (fft_Zy.real, fft_Zy.imag)                                      # Separate the real and imaginary numbers.
        Zyr = fft_Zy.real                                                       # Real numbers.
        Zyi = fft_Zy.imag                                                       # imaginary numbers.
        fft_Zx = fft(Zx, n)                                                     # FFT for East-West Deck Slope in rad/s.
        fftZx = (fft_Zx.real, fft_Zx.imag)                                      # Separate the real and imaginary numbers.
        Zxr = fft_Zx.real                                                       # Real numbers.
        Zxi = fft_Zx.imag                                                       # imaginary numbers.

        # ================================================================================================================
        # Quad Spectra with combined Zenith Deck North-South, East-West Slopes and Heave. Source: mk4 pg.30-31. Eq. (7.12)
        # ================================================================================================================
        Qn = (Zyr * Epi) - (Epr * Zyi)                                          # Quad Spectra for North-South Slope.
        Qw = (Zxr * Epi) - (Epr * Zxi)                                          # Quad Spectra for East-West Slope.

        # ================================================================================================================
        # Calculate the frequency band.
        # ================================================================================================================
        for f in range(n):
            freqband[f] = freqs[f] - freqs[f - 1]                               # Frequency band in Hz.

        # ================================================================================================================
        # Wave Direction Dm, Source: mk4 pg. 30-31. Eq. (7.26)
        # ================================================================================================================ 
            Qnv[f] = cm.phase(Qn[f])                                            # Convert real and imaginary to mag.
            Qwv[f] = cm.phase(Qw[f])                                            # Convert real and imaginary to mag.
            if Qnv[f] == 0:
                D[f] = 0
            else:
                D[f] = m.atan(-Qwv[f] / Qnv[f])                                 # Wave Direction in radians. 
            Dm[f] = m.degrees(D[f])                                             # Wave Direction in degrees.
        # ================================================================================================================

        # ================================================================================================================
        # Condition for different frequency band widths. Source: https://www.ndbc.noaa.gov/wavespectra.shtml
        # ================================================================================================================
            if freqband[f] < 0.10:
                dfn[f] = 0.005                                                  # Hz, Bandwidth of nth band frequency.
            else:
                if freqband[f] > 0.10:
                    dfn[f] = 0.01                                               # Hz, Bandwidth of nth band frequency.
                else:
                    if freqband[f] > 0.365:
                        dfn[f] = 0.02                                           # Hz, Bandwidth of nth band frequency.
        # ================================================================================================================


        # ================================================================================================================
        # Compute the Spectral Energy Density (C11) from the nondirectional wave heights. 
        # Source: NDBC Tech Doc 96-01. pg 9. or https://www.ndbc.noaa.gov/wavecalc.shtml
        # ================================================================================================================ 
        fft_theo_Epz = 2.0 * m.pi * np.abs(fft_Epz/n)                           # Calculate the theoretical Heave in m.                                 
        C11 = 2.0 * m.pi * np.abs(fft_Epz/n)**2                                 # Earth's Spectral Energy Density in m^2.
        # ================================================================================================================

        # ================================================================================================================
        # Peak or Dominant Wave Period Tp, Source: https://www.ndbc.noaa.gov/wavecalc.shtml
        # ================================================================================================================
        fp = freqs[mask][np.argmax(C11[mask])]                    # Peak frequency from the spectral wave density.
        Epzp = C11[mask].max()                                    # Pull the peak frequency at peak spectral wave density.
        Tp = 1 / fp                                               # Peak Period in seconds.
        print('Peak Wave Period =', Tp, "(sec)")
        # ================================================================================================================


        # ================================================================================================================
        # Compute the moments of the non directional wave spectra. Source: NDBC Tech Doc 96-01. pg 12.
        # ================================================================================================================
        m0 = sum(C11 * dfn)                                            # Zero moment about nondirectional wave spectra.
        m1 = sum(C11 * dfn * np.abs(freqs))                            # First moment about nondirectional wave spectra.
        m2 = sum(C11 * dfn * freqs**2)                                 # Second moment about nondirectional wave spectra.
        # ================================================================================================================


        # ================================================================================================================
        # Significant Wave Height Hs, Source: Teng 2002 - NDBC  Buoys
        # ================================================================================================================
        Hs = 4 * m.sqrt(m0)                                            # m, Significant Wave Height.
        print('Significant Wave Height =', Hs, "(m)")

        # ================================================================================================================
        # Largest Wave in sample, LWL and LWH.  Source: Teng 2002 - NDBC Buoys, Pg. 518
        # Upper and Lower Bounds calculated. 
        # ================================================================================================================
        LWL = 1.6 * Hs                                                 # m, Lower Bound of Largest Wave Height.
        print('Largest Wave Height Lower Bound =', LWL, "(m)")

        LWH = 2.0 * Hs                                                 # m, Upper Bound of Largest Wave Height.
        print('Largest Wave Height Upper Bound =', LWH, "(m)")

        # ================================================================================================================
        # Conditions to Estimate the Wave Steepness from Hs, Source: https://www.ndbc.noaa.gov/wavecalc.shtml
        # ================================================================================================================
        val = m.exp(-3.3 * m.log1p(fp))                                # Estimation Value set with Peak Wave Frequency.
        if Hs > (val / 1000):
            steepness = 'Average';                                     # First condition for average wave steepness.
        else:
            if Hs > (val / 500):
                steepness = 'Steep';                                   # Second condition for a steep wave.
            else:
                if Hs > (val / 250):
                    steepness = 'Very Steep';                          # Last condition for very steep wave.
        print('Wave Steepness =', steepness, "(m/Hz)")
        # ================================================================================================================


        # ================================================================================================================
        # Average Wave Period Ta and Zero Crossing Wave Period, Source: NDBC Tech Doc 96-01. pg 11-12.
        # ================================================================================================================
        Ta = m0 / m1                                                   # Average Wave Period in seconds.
        print('Average Wave Period =', Ta, "(sec)")

        Tz = m.sqrt(m0 / m2)                                           # Zero Crossing Wave Period in seconds.
        print('Zero Crossing Wave Period =', Tz, "(sec)")
        # ================================================================================================================


        # ================================================================================================================
        # Mean Wave Direction MWD, Source: mk4 pg. 30-31.
        # ================================================================================================================ 
        filt_D = filtfilt(b, a, Dm)                                     # Filtering Wave Direction at 1 Hz.
        MWD = np.average(filt_D)                                        # Mean Wave Direction in degrees.
        print('Mean Wave Direction =', MWD, "degrees from true North")
        # ================================================================================================================


        # ================================================================================================================
        # Filtfilt Filtering at 1 Hz of Azimuth, Azi.
        # ================================================================================================================ 
        filt_A = filtfilt(b, a, A)
        Azi = np.average(filt_A)                                        # Mean Azimuth Direction in degrees.
        print('Mean Azimuth =', Azi, "degrees from True North")
        # ================================================================================================================


        # ================================================================================================================
        # Mean IMU Temperature or Inner Buoy Temperature.
        # ================================================================================================================
        mtmp = np.average(tmp)                                          # Mean IMU/Buoy Temperature in C.
        print('Mean IMU Temperature =', mtmp, "(C)")
        # ================================================================================================================

        postdone = True

        # plt.figure(1)
        # plt.title('Heave Displacement')
        # plt.plot(freqs[mask], fft_theo_Epz[mask], label = 'Heave Theoretical FFT')
        # plt.ylabel('Heave (m)')
        # plt.xlabel('Frequency (Hz)')

        # plt.figure(2)
        # plt.title('Spectral Wave Density')
        # plt.plot(freqs[mask], C11[mask], label = 'Heave Power Spectra')
        # plt.ylabel('Spectral Wave Density (m^2)')
        # plt.xlabel('Frequency (Hz)')

        # plt.figure(3)
        # plt.plot(t, filt_A, color = 'xkcd:salmon')
        # plt.title('Azimuth')
        # plt.ylabel('Azimuth (Degrees)')
        # plt.xlabel('time (Seconds)')


        # plt.figure(4)
        # plt.plot(t, filt_D, color = 'xkcd:salmon')
        # plt.title('Wave Direction')
        # plt.ylabel('Wave Direction (Degrees)')
        # plt.xlabel('time (Seconds)')
