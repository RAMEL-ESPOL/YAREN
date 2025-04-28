#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import sys
from sensor_msgs.msg import JointState

class JointLimitsSubscriber:
    def __init__(self):
        # Inicializa el nodo de ROS
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('get_joint_limits', anonymous=True)

        # Inicializa el robot y el grupo de movimiento
        self.robot = moveit_commander.RobotCommander()
        group_name = "yaren"  # Nombre del grupo de movimiento actualizado
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Diccionario para almacenar los límites de las articulaciones
        self.joint_limits = {}

        # Obtiene los nombres de las articulaciones activas
        self.active_joints = self.move_group.get_active_joints()

        # Suscribirse al tópico /move_group/display_planned_path
        rospy.Subscriber('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, self.callback)

    def get_joint_limits(self):
        # Recorre cada articulación activa y obtiene sus límites
        for joint_name in self.active_joints:
            joint_model = self.robot.get_joint(joint_name)
            min_limit = joint_model.min_bound()
            max_limit = joint_model.max_bound()
            self.joint_limits[joint_name] = (min_limit, max_limit)
        
        # Imprime los límites de las articulaciones
        for joint, limits in self.joint_limits.items():
            rospy.loginfo("Joint: {}, Min Limit: {}, Max Limit: {}".format(joint, limits[0], limits[1]))

    def callback(self, msg):
        # Aquí se podría procesar la trayectoria planificada, pero en este caso
        # simplemente llamamos a la función que obtiene los límites de las articulaciones
        self.get_joint_limits()

if __name__ == '__main__':
    try:
        subscriber = JointLimitsSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

