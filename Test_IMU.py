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
# import os
import sys
import time
# import tsys01
import numpy as np
import datetime
import qwiic_icm20948

# =============================================================================
# import busio
# import qwiic_icm20948
# import sys
# import struct
# import math
# =============================================================================

print("Beginning IMU Test")

# Initialize Constructor
test = qwiic.QwiicTCA9548A()

# -------------------------------
# IMU SENSOR
# -------------------------------
# Take reading from IMU on Mux Channel 0
test.disable_channels([0,1,2,3,4,5,6,7])
test.enable_channels([0])
#test.list_channels()

numsamp = 20
numcols = 12 # date, time, 3 accel, 3 gyro, 3 mag, and temp
IMUdata = np.empty([numsamp,numcols]) # initialize an array for storing IMU

IMU = qwiic_icm20948.QwiicIcm20948()
 
if IMU.connected == False:
    print("The IMU is not connected.",file=sys.stderr)

IMU.begin()

for x in range(numsamp):
     if IMU.dataReady():
        IMU.getAgmt() # read all axes and temp from sensor
        
        IMUdata[x,0] = datetime.datetime.utcnow().strftime("%Y%m%d")
        IMUdata[x,1] = datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-4]
                
        # Accel (G force +/-): float(value) / scale_accel. - scale_accel takes on 16.384 (for 2g, default), 8.192 (for 4g), 4.096 (for 8g), or 2.048 (for 16g). - 3-axis MEMS accel with programmable full scale range of +/- 2g, 4g, 8g, and 16g...NOTE, this might convert to MG, not G, so the scale_accel should be multiplied by 1000 I believe.
        
        # Gyro (degrees per second): float(value) / scale_gyr. - scale_gyr takes on 131.072 (for 500dps, default), 65.536(for 1000dps), 32.768(for 2000dps), or 16.384(for 4000dps) - 3-axis MEMS gyroscope with full-scale range of +/- 250 dps, 500 dps, 1000 dps or 2000 dps. 
        
        # Mag: float(value) * 0.15 - 3-axis magnetic sensor with full-scale range of +/- 4900 micro-T
        
        # Temperature: (float(value) / 333.87 ) + 21
        
        # Apparently has on-board digital motion processor (DMP)
# =============================================================================         
#         IMUdata[x,2] = IMU.axRaw
#         IMUdata[x,3] = IMU.ayRaw
#         IMUdata[x,4] = IMU.azRaw
#         IMUdata[x,5] = IMU.gxRaw
#         IMUdata[x,6] = IMU.gyRaw
#         IMUdata[x,7] = IMU.gzRaw
#         IMUdata[x,8] = IMU.mxRaw
#         IMUdata[x,9] = IMU.myRaw
#         IMUdata[x,10] = IMU.mzRaw
#         IMUdata[x,11] = IMU.tmpRaw
# =============================================================================
        
        IMUdata[x,2] = float(IMU.axRaw) / 16.384
        IMUdata[x,3] = float(IMU.ayRaw) / 16.384
        IMUdata[x,4] = float(IMU.azRaw) / 16.384
        IMUdata[x,5] = float(IMU.gxRaw) / 131.072
        IMUdata[x,6] = float(IMU.gyRaw) / 131.072
        IMUdata[x,7] = float(IMU.gzRaw) / 131.072
        IMUdata[x,8] = float(IMU.mxRaw) * 0.15
        IMUdata[x,9] = float(IMU.myRaw) * 0.15
        IMUdata[x,10] = float(IMU.mzRaw) * 0.15
        IMUdata[x,11] = (float(IMU.tmpRaw) / 333.87) + 21
   
     else:
         IMUdata[x,:] = np.NaN # date associated with AT reading
     time.sleep(.25)
test.disable_channels([0])


print("Saving IMU data to file")
np.savetxt("IMU2.csv", IMUdata, delimiter=",")
