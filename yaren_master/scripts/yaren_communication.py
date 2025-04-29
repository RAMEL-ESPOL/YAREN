#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState

from std_msgs.msg import Header
from std_msgs.msg import Float64

# global joint_goals


joint_goals = JointState()
joint_goals.position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

def callback(data):
    joint_goals.header = Header()
    joint_goals.header.stamp = rospy.Time.now()
    joint_goals.name = data.name
    joint_goals.position = data.position
    

def main():

    rospy.init_node("yaren_communication")

    pub1  = rospy.Publisher('yaren/joint1_position_controller/command', Float64, queue_size=1)
    pub2  = rospy.Publisher('yaren/joint2_position_controller/command', Float64, queue_size=1)
    pub3  = rospy.Publisher('yaren/joint3_position_controller/command', Float64, queue_size=1)
    pub4  = rospy.Publisher('yaren/joint4_position_controller/command', Float64, queue_size=1)
    pub5  = rospy.Publisher('yaren/joint5_position_controller/command', Float64, queue_size=1)
    pub6  = rospy.Publisher('yaren/joint6_position_controller/command', Float64, queue_size=1)
    pub7  = rospy.Publisher('yaren/joint7_position_controller/command', Float64, queue_size=1)
    pub8  = rospy.Publisher('yaren/joint8_position_controller/command', Float64, queue_size=1)
    pub9  = rospy.Publisher('yaren/joint9_position_controller/command', Float64, queue_size=1)
    pub10 = rospy.Publisher('yaren/joint10_position_controller/command', Float64, queue_size=1)
    pub11 = rospy.Publisher('yaren/joint11_position_controller/command', Float64, queue_size=1)
    pub12 = rospy.Publisher('yaren/joint12_position_controller/command', Float64, queue_size=1)

    # rospy.Subscriber('joint_goals', JointState, callback)
    
    while not rospy.is_shutdown():
        rospy.Subscriber('/joint_goals', JointState, callback, queue_size=1)
        pub1.publish(joint_goals.position[0])
        pub2.publish(joint_goals.position[1])
        pub3.publish(joint_goals.position[2])
        pub4.publish(joint_goals.position[3])
        pub5.publish(joint_goals.position[4])
        pub6.publish(joint_goals.position[5])
        pub7.publish(joint_goals.position[6])
        pub8.publish(joint_goals.position[7])
        pub9.publish(joint_goals.position[8])
        pub10.publish(joint_goals.position[9])
        pub11.publish(joint_goals.position[10])
        pub12.publish(joint_goals.position[11])
        rate = rospy.Rate(10)
        

if __name__ == '__main__':
    main()