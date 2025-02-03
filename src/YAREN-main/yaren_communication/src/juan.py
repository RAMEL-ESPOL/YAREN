#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import roslib
import os
import PyKDL as kdl
import os
from kdl_parser_py.urdf import treeFromFile

# Uses Dynamixel SDK library
from motor_classes import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import * 
from getch import getch
import numpy as np
from archie_master.msg import MotorData
import math
import time


def get_positions():# Read present position
    present_positions = [0,0,0,0,0,0]   
    # Syncread present position
    dxl_comm_result = groupSyncReadPos.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("Falla en get_positions: %s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for id in range(NUM_MOTORS):
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReadPos.isAvailable(id, ADDR_PRO_PRESENT_POS , 4)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncReadPos getdata failed" % id)
        # Get Dynamixel present position value
        dxl_present_position = motor_angle_to_radian(groupSyncReadPos.getData(id, ADDR_PRO_PRESENT_POS , 4))
        present_positions[id]=dxl_present_position

    return present_positions

def get_velocities():# Read present position
    present_vel = [0,0,0,0,0,0]  
    # Syncread present position
    dxl_comm_result = groupSyncReadVel.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for id in range(NUM_MOTORS):
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReadVel.isAvailable(id, ADDR_PRO_PRESENT_VEL , 4)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % id)
        # Get Dynamixel present position value
        present_vel[id]= convert_to_rpm(groupSyncReadVel.getData(id, ADDR_PRO_PRESENT_VEL , 4))
    return present_vel


if _name_ == '_main_':

    ADDR_OPERATING_MODE         = 11               # Control table address is different in Dynamixel model
    ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_PWM           = 100
    ADDR_PRO_PRESENT_PWM        = 124
    ADDR_PRO_PRESENT_VEL        = 128
    ADDR_PRO_PRESENT_POS        = 132
    NUM_MOTORS                  = 6


    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL_ID                      = 5                 # Dynamixel ID : 5
    BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'           #'/dev/ttyUSB0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    PWM_CONTROL_MODE            = 16                         # Value for extended position control mode (operating mode)
    POS_CONTROL_MODE            = 3

    index = 0

    portHandler = PortHandler(DEVICENAME)

    packetHandler = PacketHandler(PROTOCOL_VERSION)

    start_time = time.time()

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Initialize GroupSyncWrite/Read instance
    groupSyncWritePWM       = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_PWM   , 4)
    groupSyncWritePos       = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_PRESENT_POS, 4)
    groupSyncReadPos        = GroupSyncRead (portHandler, packetHandler, ADDR_PRO_PRESENT_POS, 4)
    groupSyncReadVel        = GroupSyncRead (portHandler, packetHandler, ADDR_PRO_PRESENT_VEL, 4)

    for id in range(NUM_MOTORS):
        # Add parameter storage for Dynamixel present position value
        dxl_addparam_resultPos = groupSyncReadPos.addParam(id)
        dxl_addparam_resultVel = groupSyncReadVel.addParam(id)

        if dxl_addparam_resultPos != True:
            print("[ID:%03d] groupSyncReadPos addparam failed" % id)
        if dxl_addparam_resultVel != True:
            print("[ID:%03d] groupSyncReadVel addparam failed" % id)
        


    rospy.init_node("motor_communication_pwm")
    r =rospy.Rate(10) # 10hz

    # Set operating mode to pwm control mode
    for id in range(NUM_MOTORS):     
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode for motor", str(id), "changed to PWM control mode.")
    # Turn on motor's torque
    for id in range(NUM_MOTORS):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque of Motor",id,"is on")
    

    
    #Publish current robot state
    joint_state_pub = rospy.Publisher('/real_joint_states', JointState, queue_size=10)
    motor_state_pub = rospy.Publisher ("/motor_data" , MotorData, queue_size=1)
    subGoalState    = rospy.Subscriber('/joint_goals', JointState, callback = move_to_target, queue_size= 5)
    
    rospy.spin()    

    # Disable the torque
    for id in range(NUM_MOTORS):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque of Motor",id,"is off")

    #Setting to default control mode (Position Control Mode) 
    for id in range(NUM_MOTORS):     
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, POS_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode for motor", str(id), "changed to Position Control Mode.")

    # Clear syncread parameter storage
    groupSyncReadPos.clearParam()

    portHandler.closePort()