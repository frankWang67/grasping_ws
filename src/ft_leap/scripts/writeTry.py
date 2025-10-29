#!/usr/bin/env python
#
# ********* Gen Write Example - Modified      *********
#
# This script sets the position of a single servo motor.
# It will set the position of the servo with ID 1 to 2048.
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
#

import sys
import os
import time

sys.path.append("..")
from scservo_sdk import * # Uses FTServo SDK library


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler('/dev/ttyUSB0')# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate 1000000
if portHandler.setBaudRate(115200):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Set the position of servo with ID 1 to 2048
# The speed and acceleration are set to 0, which typically means the servo
# will move at its default or maximum speed.
scs_comm_result, scs_error = packetHandler.WritePosEx(1, 2048, 1000, 0)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
else:
    print("Successfully sent position 2048 to servo ID 1")

# Close port
portHandler.closePort()
