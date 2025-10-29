#!/usr/bin/env python
#
# ********* Sync Write Example - Modified      *********
#
# This script sets the position for multiple servos simultaneously.
# It will set the position of all servos with IDs from 0 to 15 to 2048.
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

# Set port baudrate to 115200
if portHandler.setBaudRate(115200):
    print("Succeeded to change the baudrate to 115200")
else:
    print("Failed to change the baudrate")
    quit()

# Add servo(id)#0~15 goal position to the Syncwrite parameter storage
for scs_id in range(0, 16):
    # Sets the target position to 2048 with a default speed and acceleration
    scs_addparam_result = packetHandler.SyncWritePosEx(scs_id, 2048, 60, 50)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % scs_id)

# Syncwrite goal position to all servos
scs_comm_result = packetHandler.groupSyncWrite.txPacket()
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
else:
    print("Successfully sent position command to all servos.")

# Clear syncwrite parameter storage
packetHandler.groupSyncWrite.clearParam()

# Close port
portHandler.closePort()
