

# UNIVERSITY MINNESOTA DULUTH DATA AQUISITION BUOY

This code is the data aquisition and and satellite trasmission code for the DAQ Buoy developed by UMD senior design and Dr. Craig Hill. The code
incorporates sensors like temp, IMU, gps and transmits via a sattellite link using a raspberry pi.

BuoyDAQ4.py is the main code updated 4/20/2021

#### All other files are tests or example data:
* 9603_Test.py - Satellite test
* Test_ATWT.py - Air and water temperature tests
* i2c_test.py  - test i2c extention board
* GPS_Test.py - GPS test (Do not use Test_GPS.py)
* Test_IMU - IMU test will save data to csv file

Main code will run in a 1 hour loop collecting GPS data as well as Air and water temperature data every minute. IMU data collection will be collected for
20 minutes ever hour once and hour. This data will be processed and a readout of important wave data will be saved and output. Every 10 minutes important data such as
GPS, temp data,time, day, location, and small information about wave will be sent via satellite to corresponding satellite account.
