#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import sys

def move_group_python_interface():
    # Inicializa el nodo
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # Inicializa el robot, el grupo de movimiento y la escena
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "yaren"  # Nombre del grupo de movimiento actualizado
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Inicializa el publisher para el tópico display_planned_path
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Define la posición objetivo para los 12 joints
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.0  # Ejemplo de posición para el joint 1
    joint_goal[1] = 0.0  # Ejemplo de posición para el joint 2
    joint_goal[2] = 0.0  # Ejemplo de posición para el joint 3
    joint_goal[3] = 0.0  # Ejemplo de posición para el joint 4
    joint_goal[4] = 0.0  # Ejemplo de posición para el joint 5
    joint_goal[5] = 0.0  # Ejemplo de posición para el joint 6
    joint_goal[6] = 0.0  # Ejemplo de posición para el joint 7
    joint_goal[7] = 1.0  # Ejemplo de posición para el joint 8
    joint_goal[8] = 0.0  # Ejemplo de posición para el joint 9
    joint_goal[9] = 0.0  # Ejemplo de posición para el joint 10
    joint_goal[10] = 0.0  # Ejemplo de posición para el joint 11
    joint_goal[11] = 1.0  # Ejemplo de posición para el joint 12

    # Obtiene los límites de los joints desde el modelo del robot
    active_joints = move_group.get_active_joints()
    joint_min_limits = []
    joint_max_limits = []

    for joint_name in active_joints:
        joint_model = robot.get_joint(joint_name)
        joint_min_limits.append(joint_model.min_bound())
        joint_max_limits.append(joint_model.max_bound())

    # Verifica que los valores de los joints estén dentro de los límites
    for i in range(len(joint_goal)):
        if joint_goal[i] < joint_min_limits[i] or joint_goal[i] > joint_max_limits[i]:
            rospy.logerr("Joint value for joint {} is out of bounds: {}".format(active_joints[i], joint_goal[i]))
            return

    # Intenta establecer el objetivo de los joints
    try:
        move_group.set_joint_value_target(joint_goal)
    except moveit_commander.MoveItCommanderException as e:
        rospy.logerr("Error setting joint target: {}".format(e))
        return

    # Planifica la trayectoria
    plan = move_group.plan()
    if not plan:
        rospy.logerr("Planning failed")
        return

    # Ejecuta la trayectoria planificada
    success = move_group.go(wait=True)
    if not success:
        rospy.logerr("Execution failed")
        return

    move_group.stop()  # Asegura que no haya movimiento residual

    # Crea el mensaje DisplayTrajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publica la trayectoria planificada en el tópico /move_group/display_planned_path
    display_trajectory_publisher.publish(display_trajectory)

    # Cierra y limpia
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_group_python_interface()
    except rospy.ROSInterruptException:
        pass

