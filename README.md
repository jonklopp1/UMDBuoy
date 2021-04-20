

# UNIVERSITY MINNESOTA DULUTH DATA AQUISITION BUOY

This code is the data aquisition and and satellite trasmission code for the DAQ Buoy developed by UMD senior design and Dr. Craig Hill. The code
incorporates sensors like temp, IMU, gps and transmits via a sattellite link using a raspberry pi.

#### Main Code:
BuoyDAQ4.py is the main code updated 4/20/2021

Use:
~~~
python BuoyDAQ4.py
~~~
To run the code when all applicable libraries are installed and sensors are in a working condition

#### All other files are tests or example data:
* 9603_Test.py - Satellite test
* Test_ATWT.py - Air and water temperature tests
* i2c_test.py  - test i2c extention board
* GPS_Test.py - GPS test (Do not use Test_GPS.py)
* Test_IMU - IMU test will save data to csv file

Main code will run in a 1 hour loop collecting GPS data as well as Air and water temperature data every minute. IMU data collection will be collected for
20 minutes ever hour once and hour. This data will be processed and a readout of important wave data will be saved and output. Every 10 minutes important data such as
GPS, temp data,time, day, location, and small information about wave will be sent via satellite to corresponding satellite account.

#### Sensors used:
 * Blue Robotics Temerature sensor CELSIUS-SENSOR-R1-RP (and external antenna)
 * Adafruit Ultimate GPS
 * RockBlock 9603 Satellite 14498 (and external antenna)
 * 9 DOF ICM - 20948 IMU
 * SparkFun Qwiic Mux Breakout Multiplexer (extra i2c Slots)
 * Raspberry Pi 3





## Installalation Guide

This guide will describe the step-by-step process required for setting up a Raspberry Pi in the Drifting Data Acquisition Buoy, which will be responsible for handling sampling rates, interpreting sensor data, logging necessary information, and transmitting relevant data to the user.

An operating system may need to be installed onto the MicroSD card, if this is the case, please complete the following steps.

Step 1) First, download the version of Raspbian you’d like to install (options are Raspbian and Raspbian Lite, both will work).

	Step 2) Once downloaded, unzip the zip file, you’ll be left with a .img file.

Step 3) Next, download & install software that will write the image to the SD card. Etcher is recommended as it works on all platforms.

	Step 4) Open Etcher and select the Raspbian image.

	Step 5) Choose the SD card that you’ll be using to install Raspbian on.

	Step 6) Proceed to flash the SD card, select Flash.

	Step 7) Remove the SD card from the computer.

Step 8) Raspbian should be installed on the SD card and can be inserted into the Raspberry Pi.

To install the memory card, simply insert a MicroSD card (minimum of 16GB) into the available memory slot on the Raspberry Pi. To ensure proper functionality of the Raspberry Pi, please ensure all wiring is correct by referring to the “Electrical Schematics” section and the Assembly Guide.

It is assumed that the Raspberry Pi has already been hooked up to a power supply, Pi has been connected to a wireless network, and has either been connected to via SSH or a keyboard and monitor have been hooked up directly to the system. Instructions will be given as command line prompts that will need to be entered.

Part 1: Update/Upgrade the Raspberry Pi

Type the following code line into the command line and then hit enter. This command will fetch updates for the Raspbian OS. It may take a few minutes for updates to be retrieved.
sudo apt-get update

Type the following code line into the command line and then hit enter. After entering the command there will be a prompt asking if the user is sure they want to install the new upgrades. Type ‘y’ for yes and hit enter again; updates will begin to be applied. It may take a few minutes for upgrades to be installed.
sudo apt-get upgrade

It may be required to run the Step 1 commands multiple times in case of a failed download/installation. If there have been no changes made and no new updates, the system has been properly updated and you may proceed to part 2.

Part 2: Install/Update Python 3

Type in the following code line into the command line and then hit enter. This command will install Python 3 if it is not already present on the Raspberry Pi. If Python 3 was previously installed, this command will fetch any updates available for the present installation.
sudo apt-get install python3-pip

Part 3: Confirm that I2C and SPI are enabled

Type in the following code line into the command line and then hit enter.
ls /dev/i2c* /dev/spi*

If I2C and SPI have been enabled appropriately, you should see: 
/dev/i2c-1 /dev/spidev0.0 /dev/spidev0.1

Part 4: (OPTIONAL) Enable Second SPI (If required for additional sensors)

Type in the following code line into the command line and then hit enter. This command will open up the config.txt file in the text editor nano. 
sudo nano /boot/config.txt

Using the down arrow key, scroll to the very bottom of the config.txt file. Add the following code line at the very bottom of the config document.
dtoverlay=spi1-3cs

Next hit “Ctrl+X” to exit the file editor. A prompt will ask if the file should be saved, enter ‘y’ then hit the enter key. Now that the changes have been saved; it is time to reboot the Raspberry Pi. To do this type in the following code line into the command line and then hit enter.
sudo reboot

Once the system has finished rebooting; sign back into the pi and type in the code text below into the command line and hit enter.
ls /dev/i2c* /dev/spi*

Now that the second SPI has been enabled; there should be a response that looks like:
 /dev/spidev0.0 /dev/spidev0.1 /dev/spidev1.0 /dev/spidev1.1 /dev/spidev1.2

Part 5: Installing Additional Python 3 libraries

Type in the following code text into the command line and then hit enter. This Python library makes working with the GPIO ports on the Raspberry Pi easier.
sudo pip3 install RPI.GPIO

Next type in the following code text into the command line and then hit enter. This library will be used with all Adafruit components.
sudo pip3 install adafruit-blinka

Part 6: Set Python 3 as default Python version

Python 2 is the default version of Python that is used in Raspbian; however Python 3 is required for all adafruit libraries. Begin by checking the current version of python on the Raspberry Pi, this is done by entering the code text below.
python --version

If this returns version 3.#.# then Python 3 is already the default programming language and it is possible to simply skip to step 7. Otherwise bashrc must be modified; this can be done by first entering the code text below.
sudo nano ~/.bashrc

Now using the down arrow scroll all the way to the bottom of the file, where the following code text should be entered at the end of the file.
alias python=’/usr/bin/python3’

To exit the file editor hit “Ctrl+X”. When prompted enter ‘y’, then hit the enter key. The file bashrc has now been edited and saved. To apply this change to the system without rebooting the Raspberry Pi enter the following code text.
source ~/.bashrc

Finally to check that the appropriate changes have been made, check the version of python again. Do this by entering the following code text.
python --version

The returned version should now appear as a variation of 3.#.#. Setting Python 3 as the default version of python is now complete.

Part 7: Set pip to work for Python 3
Begin by entering the file editor for .bashrc:
sudo nano ~/.bashrc

Now at the bottom of the document add the line:
alias pip=pip3

Exit the file by hitting ‘Ctrl+x’, hit ‘y’, and then finally hit the ‘enter’ key. This should save the changes you’ve made to .bashrc. Lastly enter the following command to apply this change without needing to restart the Raspberry Pi.
source ~/.bashrc

Part 8: Install individual component libraries
Assuming everything has been properly installed before this (Adafruit Blinka installed, I2C enabled, and Python 3 being used). We can now install individual adafruit component libraries to the Raspberry Pi.

Step 1) To install the Qwiic-icm20948 IMU library type in the following code into the command line:
sudo pip install sparkfun-qwiic-icm20948
	
Step 2) To install the Ultimate GPS library type in the following code into the command line:
sudo pip3 install adafruit-circuitpython-gps

Step 3) To install the tsys01 library (for Celsius Fast-Response Temperature Sensor) first go to the following file location:
cd /usr/local/lib/python3.7/dist-packages/

type in the following commands into the command line:
Git clone https://github.com/bluerobotics/tsys01-python
	Then change directories with:
cd tsys01-python
Go to the tsys01-python folder and install the appropriate library by entering the following command:
sudo python setup.py

Step 4) In order to complete the tsys01 library install correctly, install the smbus2 library with the following command:
sudo pip3 install smbus2

Next access the tsys01 python file with the following command:
sudo nano tsys01.py

Within the tsys01 python file, change every occurrence of “smbus” to “smbus2”. This makes tsys01.py work with python 3.

