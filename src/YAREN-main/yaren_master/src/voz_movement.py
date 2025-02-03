#!/usr/bin/env python3
#MOVEMENT
import rospy
#MOVEMENT
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String

#VOZ
import sys
import os
import json
import ast


#number='0'

class movement:
    def __init__(self):
        rospy.init_node("voz_motion_control")
        self.r = rospy.Rate(10)
        self.pub_joint_states = rospy.Publisher('/joint_goals', JointState, queue_size=1)
        self.sub_commands = rospy.Subscriber('/movement_commands', String, self.callback_commands)

        self.joints_states = JointState()
        self.joints_states.header = Header()
        self.joints_states.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
                                   "joint_6", "joint_7", "joint_8", "joint_9", "joint_10",
                                   "joint_11", "joint_12"]

        self.number = "0"

    def loop(self):
        #self.handle_client_connection()        
        self.main()
        self.r.sleep()


    def callback_commands(self, msg):
        try:
            self.number = msg.data
            print(f"Recibido: {self.number}")
            
        except ValueError:
            rospy.logerr("Error al procesar el mensaje recibido.")

    def process_file(self,file_path):
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()

            for i, line in enumerate(lines):
                try:
                    # Separar la posición del tiempo
                    parts = line.strip().split(";")
                    positions = ast.literal_eval(parts[0].strip())
                    time = float(parts[1].strip()) if len(parts) > 1 else 1.0

                    # Validar formato
                    if not isinstance(positions, list) or not all(isinstance(x, (int, float)) for x in positions):
                        raise ValueError("Formato inválido de posiciones")

                    # Crear mensaje y publicar
                    #self.joint_position_state=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                    #self.joints_states.position = self.joint_position_state
                    ##self.pub_joint_states.publish(self.joints_states)
                    #self.pub_joint_states.publish(self.joints_states)
                    #print("Base")
                    #rospy.sleep(time)

                    self.joint_position_state = positions
                    self.joints_states.position = self.joint_position_state
                    self.pub_joint_states.publish(self.joints_states)

                    # Imprimir y esperar el tiempo especificado
                    print(f"Línea {i+1}: Posiciones {positions}, Tiempo {time}s")
                    rospy.sleep(time)

                except Exception as e:
                    print(f"Error ejecutando línea {i+1}: {line.strip()}, {e}")

        except FileNotFoundError:
            print(f"El archivo {file_path} no existe.")
        except Exception as e:
            print(f"Error procesando el archivo: {e}")
    

    def main(self):
 

        g30=0.5235987756
        g45=0.7853981634

        time=1
        walkingtime=0.5

        
        self.number


        while not rospy.is_shutdown():
            


            if (self.number=='1'):
            
                self.joint_position_state=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub_joint_states.publish(self.joints_states)
                self.pub_joint_states.publish(self.joints_states)
                print("Base")
                rospy.sleep(time)

            elif (self.number=='2'):
            
                file_path = os.path.dirname(os.path.abspath(__file__)) + "/abrazo.txt"
                self.process_file(file_path)

            elif (self.number=='3'):
            
                self.joint_position_state=[-0.3,-0.69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub_joint_states.publish(self.joints_states)
                self.pub_joint_states.publish(self.joints_states)
                rospy.sleep(time)

            elif (self.number=='4'):
            
                self.joint_position_state=[0.7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub_joint_states.publish(self.joints_states)
                self.pub_joint_states.publish(self.joints_states)
                rospy.sleep(time)
            
            elif (self.number=='5'):
            
                while not rospy.is_shutdown():
                    file_path = os.path.dirname(os.path.abspath(__file__)) + "/luchaLlamado.txt"
                    self.process_file(file_path)
            
            elif (self.number=='6'):

                while not rospy.is_shutdown():
                    self.joint_position_state=[0.0,0.0,0.0,0.0,0.78,0.0,0.0,0.52,0.0,0.52,0.52,0.0]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.4,0.0,0.0,0.0,0.78,0.0,0.0,0.52,0.0,0.52,0.52,0.0]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.4,0.0,0.0,0.0,0.78,0.52,0.52,0.52,-0.78,0.52,-0.52,0.52]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.0,0.0,0.0,0.0,0.78,0.52,0.52,1.5,-0.78,0.52,-0.52,1.5]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.0,0.0,0.0,0.0,0.78,0.52,0.52,0.52,-0.78,0.52,-0.52,0.52]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.0,0.0,0.0,0.0,0.78,0.52,0.52,1.5,-0.78,0.52,-0.52,1.5]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.0,0.0,0.0,0.0,0.78,0.52,-0.52,1.5,-0.78,0.52,0.52,1.5]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.0,0.0,0.0,0.0,0.78,0.52,0.52,1.5,-0.78,0.52,-0.52,1.5]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)

                    self.joint_position_state=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.48,0.0,0.0,0.0,0.48]
                    self.joints_states.position = self.joint_position_state
                    #self.pub_joint_states.publish(self.joints_states)
                    self.pub_joint_states.publish(self.joints_states)
                    rospy.sleep(time)
            
            elif (self.number=='7'):
                while not rospy.is_shutdown():
                    file_path = os.path.dirname(os.path.abspath(__file__)) + "/baile_de_barney.txt"
                    self.process_file(file_path)

            time=1
                    




if __name__ == '__main__':
    movement= movement()
    while not rospy.is_shutdown():
        movement.loop()  
