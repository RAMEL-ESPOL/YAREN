#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String

class movement:
    def __init__(self):
        rospy.init_node("motion_control")
        self.r = rospy.Rate(10)
        self.pub = rospy.Publisher('/joint_goals', JointState, queue_size=1)
       

    def loop(self):
        self.main()
        self.r.sleep()


    def main(self):

        g30=0.5235987756
        g45=0.7853981634

        time=1
        walkingtime=0.5

        self.joints_states = JointState()
        self.joints_states.header = Header()
        self.joints_states.header.stamp = rospy.Time.now()
        self.joints_states.name = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7","joint_8","joint_9","joint_10","joint_11","joint_12"]

        while not rospy.is_shutdown():
            
            number = input ("Enter number: ")
           

            if (number=='1'):
            
                self.joint_position_state=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                print("Base")
                rospy.sleep(time)

            if (number=='2'):
            
                self.joint_position_state=[0.3,0.69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                rospy.sleep(time)

            if (number=='3'):
            
                self.joint_position_state=[-0.3,-0.69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                rospy.sleep(time)

            if (number=='4'):
            
                self.joint_position_state=[0.7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
            
            if (number=='5'):
            
                self.joint_position_state=[0.0,0.0,0.0,0.0,0.78,0.0,0.0,0.52,0.0,0.52,0.52,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                rospy.sleep(time)



if __name__ == '__main__':
    movement= movement()
    while not rospy.is_shutdown():
        movement.loop()