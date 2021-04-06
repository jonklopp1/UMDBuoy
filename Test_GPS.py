# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 15:51:44 2020

@author: cshill
"""

# =============================================================================
# from typing import Any
# from __future__ import print_function
# import time
# import qwiic
# import qwiic_icm20948
# import sys
# import os
# from time import sleep
# from datetime import datetime
# import board
# import busio
# from adafruit_lsm6ds import LSM6DS33
# import RPi.GPIO as GPIO
# import tsys01
# import glob
# import signal
# import math
# =============================================================================

import adafruit_gps
import serial
import time
import numpy as np

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

# -------------------------------
# GPS DATA
# -------------------------------

numsamp = 10
numcols = 8
# Column 1 = yyyy
# Column 2 = mm
# Column 3 = dd
# Column 4 = hr
# Column 5 = min
# Column 6 = sec
# Column 7 = Lat
# Column 8 = Lon

GPSdata = np.empty([numsamp,numcols]) # initialize an array for storing IMU
 
# Checks for new GPS data
gps.update()

for x in range(numsamp):
    
    print("Pausing 10 seconds...")
    print("...")
    print("...")
    time.sleep(10)
    
    # GPS Data is gathered
    if not gps.has_fix: 
        # No GPS Signal detected
        print("No GPS signal...")
        gps_yyyy = np.NaN
        gps_mm = np.NaN
        gps_dd = np.NaN
        gps_HH = np.NaN
        gps_MM = np.NaN
        gps_SS = np.NaN
        gps_lat = np.NaN
        gps_lon = np.NaN
        
    else:
        # GPS Signal detected
        print("GPS acquired...")
        gps_yyyy = gps.timestamp_utc.tm_year
        gps_mm = gps.timestamp_utc.tm_mon
        gps_dd = gps.timestamp_utc.tm_mday
        gps_HH = gps.timestamp_utc.tm_hour
        gps_MM = gps.timestamp_utc.tm_min
        gps_SS = gps.timestamp_utc.tm_sec
        gps_lat = gps.latitude
        gps_lon = gps.longitude
        
# =============================================================================
#         lat = '{0:.5f}'.format(gps.latitude) # .5 gives 3ft accuracy, .6 gives 3inch accuracy
#         long = '{0:.5f}'.format(gps.longitude)
# =============================================================================

    GPSdata[x,0] = gps_yyyy
    GPSdata[x,1] = gps_mm
    GPSdata[x,2] = gps_dd
    GPSdata[x,3] = gps_HH
    GPSdata[x,4] = gps_MM
    GPSdata[x,5] = gps_SS
    GPSdata[x,6] = gps_lat
    GPSdata[x,7] = gps_lon
    
print("Saving GPS data to file")
np.savetxt("GPS.csv", GPSdata, delimiter=",")    