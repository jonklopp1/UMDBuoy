#-----------------------------------------------------------------------
# Qwiic Mux - Example 1
#-----------------------------------------------------------------------
#
# Written by  SparkFun Electronics, June 2019
# Author: Wes Furuya
#
# Compatibility: https://www.sparkfun.com/products/14685
# 
# Do you like this library? Help support SparkFun. Buy a board!
# For more information on Qwiic Mux, check out the product page
# linked above.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http:www.gnu.org/licenses/>.
#
#=======================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#=======================================================================

"""
This example enables channels 0 and 4, pauses, and then enables channel 7.
"""

from __future__ import print_function
import qwiic
import os
import time
import tsys01
import numpy as np
import datetime

# =============================================================================
# import busio
# import qwiic_icm20948
# import sys
# import struct
# import math
# =============================================================================

# Initialize Constructor
test = qwiic.QwiicTCA9548A()

# Test Run
#########################################
# -------------------------------
# IMU SENSOR
# -------------------------------
# Take reading from IMU on Mux Channel 0
# =============================================================================
# test.disable_channels([0,1,2,3,4,5,6,7])
# =============================================================================
# =============================================================================
# test.enable_channels([0])
# # List Channel Configuration
# test.list_channels()
# 
# def runExample():
# 
#     print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
#     IMU = qwiic_icm20948.QwiicIcm20948()
# 
#     if IMU.connected == False:
#         print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
#             file=sys.stderr)
#         return
# 
#     IMU.begin()
# 
# #    while True:
#     for x in range(10):
#         if IMU.dataReady():
#             IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
#             print(\
#                     '{: 06d}'.format(IMU.axRaw)\
#             , '\t', '{: 06d}'.format(IMU.ayRaw)\
#             , '\t', '{: 06d}'.format(IMU.azRaw)\
#             , '\t', '{: 06d}'.format(IMU.gxRaw)\
#             , '\t', '{: 06d}'.format(IMU.gyRaw)\
#             , '\t', '{: 06d}'.format(IMU.gzRaw)\
#             , '\t', '{: 06d}'.format(IMU.mxRaw)\
#             , '\t', '{: 06d}'.format(IMU.myRaw)\
#             , '\t', '{: 06d}'.format(IMU.mzRaw)\
#             , '\t', '{: 06d}'.format(IMU.tmpRaw)\
#             )
#             time.sleep(0.1)
#         else:
#             print("Waiting for IMU data")
#             time.sleep(1)
# 
# if __name__ == '__main__':
#     try:
#         runExample()
#     except (KeyboardInterrupt, SystemExit) as exErr:
#         print("\nEnding Example 1")
#         sys.exit(0)
# 
# =============================================================================
# Create variable for Temperature sensors
AWTdata = np.empty([100,6]) # initialize an array for storing AT and WT data
# Column 1 = yyyyddmm (string) - AT
# Column 2 = hhmmss.ss (string) - AT
# Column 3 = AT
# Column 4 = yyyyddmm (string) - WT
# Column 5 = hhmmss.ss (string) - wT
# Column 6 = WT

# Create AWT file
#global file_path
#global filename

#def log_setup():
#    filename = "AWT_Data"
#    file_path = "~/" + filename + ".csv" # saves in the root directory
#    temp_file = open(file_path, "a") # opens file object for appending data
#    if os.stat(file_path).st_size == 0:
#        usr_input = ["Date (YYYYMMDD)", "Time (hhmmss.ss)", "AT (C)", "Date (YYYYMMDD)", "Time (hhmmss.ss)", "WT (C)"]
#        temp_file.write(usr_input + "\n") # writes headers to file 
#    time.sleep(5)
#    temp_file.close()
#    return file_path

# -------------------------------
# AIR TEMPERATURE SENSOR
# -------------------------------
test.disable_channels([0,1,2,3,4,5,6,7])

test.enable_channels([1])
AT = tsys01.TSYS01()
AT.init()
print("AT sensor initialized")
test.disable_channels([1])

test.enable_channels([2])
WT = tsys01.TSYS01()
WT.init()
print("WT sensor initialized")
test.disable_channels([2])

# List Channel Configuration
# test.list_channels()

# Take readings

for x in range(100):

   test.enable_channels([1]) 
   if not AT.read():
        AWTdata[x,0] = np.NaN # date associated with AT reading
        AWTdata[x,1] = np.NaN # time associated with AT reading
        AWTdata[x,2] = np.NaN # AT data
        print("Error - AT sensor")
   else:       
        AWTdata[x,0] = datetime.datetime.utcnow().strftime("%Y%m%d")
        AWTdata[x,1] = datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-4]
        AWTdata[x,2] = "%.2f" % AT.temperature(tsys01.UNITS_Centigrade)
# =============================================================================
#         print("Successful AT reading")
# =============================================================================
        
   test.disable_channels([1])
   
   test.enable_channels([2])
   if not WT.read():
        AWTdata[x,3] = np.NaN # date associated with WT reading
        AWTdata[x,4] = np.NaN # time associated with WT reading
        AWTdata[x,5] = np.NaN # WT data
        print("Error - WT sensor")
   else:
        AWTdata[x,3] = datetime.datetime.utcnow().strftime("%Y%m%d")
        AWTdata[x,4] = datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-4]
        AWTdata[x,5] = "%.2f" % WT.temperature(tsys01.UNITS_Centigrade)
# =============================================================================
#         print("Successful WT reading")
# =============================================================================
        
   test.disable_channels([2])
   
#  time.sleep(0.5)
   print("Saving ATWT data to file")
   np.savetxt("AWT.csv", AWTdata, delimiter=",")
   
   # Log Data
#   data_File = open("~/AWT_Data.csv", "a")
#   data_File.write(AWTdata + "\n")
#   data_File.flush()
#   time.sleep(10)
#   data_File.close()
