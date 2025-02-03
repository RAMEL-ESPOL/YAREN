#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String

import cv2
import mediapipe as mp
import numpy as np
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose


class kinematic:
    def __init__(self):
        rospy.init_node("kinematic_motion_control")
        self.r = rospy.Rate(10)
        self.pub = rospy.Publisher('/joint_goals', JointState, queue_size=1)
       

    def loop(self):
        self.main()
        self.r.sleep()
    
    def calculate_angle(a,b,c):
        a = np.array(a) # First
        b = np.array(b) # Mid
        c = np.array(c) # End
        
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
        
        if angle >180.0:
            angle = 360-angle
            
        return angle-90.0 


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
            
            cap = cv2.VideoCapture(-1)
            ## Setup mediapipe instance
            with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
                while cap.isOpened():
                    ret, frame = cap.read()
                    
                    # Recolor image to RGB
                    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGR)
                    image.flags.writeable = False
                
                    # Make detection
                    results = pose.process(image)
                
                    # Recolor back to BGR
                    image.flags.writeable = True
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    
                    # Extract landmarks
                    try:
                        landmarks = results.pose_landmarks.landmark
                        
                        # Get coordinates
                        hip_r = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
                        shoulder_r = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
                        elbow_r = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
                        wrist_r = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
                        
                        hip_l = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
                        shoulder_l = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
                        elbow_l = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
                        wrist_l = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
                        
                        # Calculate angle
                        angle_elbow_l = calculate_angle(shoulder_l, elbow_l, wrist_l)
                        angle_should_r= calculate_angle(shoulder_r, shoulder_l, elbow_l)
                        print(angle_should_r)


                        self.joint_position_state=[0.0,0.0,0.0,0.0,angle_should_r,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                        self.joints_states.position = self.joint_position_state
                        #self.pub.publish(self.joints_states)
                        self.pub.publish(self.joints_states)
                        rospy.sleep(time)
                        
                        # Visualize angle
                        cv2.putText(image, str(angle_elbow_l), 
                                    tuple(np.multiply(elbow_l, [640, 480]).astype(int)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                            )
                        
                        cv2.putText(image, str(angle_should_r), 
                                    tuple(np.multiply(shoulder_l, [640, 480]).astype(int)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                            )
                                
                    except:
                        pass
                    
                    
                    # Render detections
                    mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                            mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                                            mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2) 
                                            )               
                    
                    cv2.imshow('Mediapipe Feed', image)

                    if cv2.waitKey(10) & 0xFF == ord('q'):
                        break

                cap.release()
                cv2.destroyAllWindows()
           


if __name__ == '__main__':

    kinematic = kinematic()

    while not rospy.is_shutdown():

        kinematic.loop()