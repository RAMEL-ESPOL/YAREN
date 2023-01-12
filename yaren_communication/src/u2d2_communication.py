#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import roslib
import os
# Uses Dynamixel SDK library
from motor_classes import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import * 

value_angles_max = []
value_angles_min = []

def set_positions(data,callback_args):
    list_motors = callback_args[0]
    bool_init = callback_args[1]
    num_joints = callback_args[2]
    joint_state_pub = callback_args[3]
    print("Entro a set_positions")

    """for id in range(len(data.position)):
        pos = data.position[id]
        print(pos)"""
        #if pos > value_angles_max[id]: data.position[id] = value_angles_max[id]
        #elif pos < value_angles_min[id]: data.position[id] = value_angles_min[id]
    
    for motor in list_motors:
        print("Lista de motores")
        print(motor.list_ids)
        for id in motor.list_ids:
            print("ID:")
            print(id)
            #Check if zero value positions are the initial positions for each joint!!!!!!!!!!!!!!!!!!!!!!!!
            if bool_init:
                dxl_comm_result, dxl_error = motor.packetHandler.write4ByteTxRx(motor.portHandler, id, motor.addr_goal_position, motor.angle_zero)
                print("Dynamixel has successfully set the initial position")
            else:
                new_angle = motor.angleConversion(data.position[id-1], False,id)
                print(new_angle)
                dxl_comm_result, dxl_error = motor.packetHandler.write4ByteTxRx(motor.portHandler, id, motor.addr_goal_position, new_angle)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % motor.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % motor.packetHandler.getRxPacketError(dxl_error))
    
    print("for motor!")

    #joint_state_publisher(list_motors,num_joints,joint_state_pub)



####
def get_positions(list_motors):# Read present position
    for motor in list_motors:
        for id in motor.list_ids:
            dxl_present_position, dxl_comm_result, dxl_error = motor.packetHandler.read4ByteTxRx(motor.portHandler, id, motor.addr_present_position)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % motor.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % motor.packetHandler.getRxPacketError(dxl_error))
            ##################
            general_joint_position[id]=dxl_present_position




##########
def joint_state_publisher(list_motors,num_joints,joint_state_pub):
    joints_states = JointState()
    joints_states.header = Header()
    joints_states.header.stamp = rospy.Time.now()
    joints_states.name = ['joint_'+str(id+1) for id in range(num_joints)]
    #Read actual position after movement occured
    general_joint_position = get_positions(list_motors)
    #Convert from 0-4095 to degrees
    print("Joint State")
    for motor in list_motors:
        for id in motor.list_ids:
            general_joint_position_state[id-1]=motor.angleConversion(general_joint_position[id-1],True,id) 
    #Publish the new joint state
    joints_states.position = general_joint_position_state
    joints_states.velocity = []
    joints_states.effort = []
    joint_state_pub.publish(joints_states)



if __name__ == '__main__':

    rospy.init_node("yaren_motor_communication")
    r =rospy.Rate(10) # 10hz

    usb_port = rospy.get_param('~usb_port')
    dxl_baud_rate = rospy.get_param('~dxl_baud_rate')


    num_joints = 12
    general_joint_position = [0 for i in range(num_joints)]
    general_joint_position_state = [0 for i in range(num_joints)]

    portHandler = PortHandler(usb_port)
    packetHandler = PacketHandler(2.0)

    
    

    #Last value is the max desired speed: value*0.229rpm is the speed in rpm
    
    neck_motor = XCseries_motor(usb_port,dxl_baud_rate,[3,4],portHandler,packetHandler,r,20,{3:[-1,1],4:[-1,1]},{3:[100,0,0],4:[100,0,0]})

    shoulder_right_motor = XCseries_motor(usb_port,dxl_baud_rate,[5],portHandler,packetHandler,r,20,{5:[-1,1]},{5:[100,0,0]})
    forearm_right_motor = XCseries_motor(usb_port,dxl_baud_rate,[6,7],portHandler,packetHandler,r,20,{6:[-1,1],7:[-1,1]},{6:[100,0,0],7:[100,0,0]})
    elbow_right_motor = XCseries_motor(usb_port,dxl_baud_rate,[8],portHandler,packetHandler,r,20,{8:[-1,1]},{8:[100,0,0]})

    shoulder_left_motor = XCseries_motor(usb_port,dxl_baud_rate,[9],portHandler,packetHandler,r,20,{9:[-1,1]},{9:[100,0,0]})
    forearm_left_motor = XCseries_motor(usb_port,dxl_baud_rate,[10,11],portHandler,packetHandler,r,20,{10:[-1,1],11:[-1,1]},{10:[100,0,0],11:[100,0,0]})
    elbow_left_motor = XCseries_motor(usb_port,dxl_baud_rate,[12],portHandler,packetHandler,r,20,{12:[-1,1]},{12:[100,0,0]})

    trunk_motor = MX_motor(usb_port,dxl_baud_rate,[1],portHandler,packetHandler,r,20,{1:[-1,1]},{1:[100,0,0]})
    hip_motor = MX_motor(usb_port,dxl_baud_rate,[2],portHandler,packetHandler,r,20,{2:[-1,1]},{2:[100,0,0]})

    list_motors = [neck_motor,shoulder_right_motor,forearm_right_motor,elbow_right_motor,
    shoulder_left_motor,forearm_left_motor,elbow_left_motor,trunk_motor,hip_motor]

    #list_motors = [shoulder_right_motor,forearm_right_motor,forearm_right_motor]

    #Publish current robot state
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    set_positions({},[list_motors,True,num_joints,joint_state_pub])

    # Subscribe desired joint position
    rospy.Subscriber('/joint_goals', JointState,set_positions,(list_motors,False,num_joints,joint_state_pub), queue_size=5)
    print("subcribir")

    while not rospy.is_shutdown():  
        print("Antes del Spin")
        rospy.spin()
        print("Escuchando...")
        #joint_state_publisher(list_motors,num_joints,joint_state_pub)
        r.sleep()
    portHandler.closePort()



    



    
        



    
