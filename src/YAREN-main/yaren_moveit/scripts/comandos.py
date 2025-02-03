#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import sys
from moveit_commander.exception import MoveItCommanderException

def move_group_python_interface(pose):
    # Inicializa el nodo
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # Inicializa el robot, el grupo de movimiento y la escena
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "yaren"  # Nombre del grupo de movimiento actualizado
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Configura la velocidad y aceleración máximas
    move_group.set_max_velocity_scaling_factor(0.5)
    move_group.set_max_acceleration_scaling_factor(0.5)

    # Inicializa el publisher para el tópico display_planned_path
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Define las posiciones objetivo predefinidas dentro de los límites
    poses = {
        'a': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5],  # Gesto de pedir caridad
        'b': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.7, 0.2, 0.2, 0.2, 0.7],  # Gesto de abrazo
        'c': [0.1, -0.1, 0.1, -0.1, 0.1, 0.1, 0.1, 0.2, 0.1, 0.1, 0.1, 0.2],  # Gesto de sorprendido
        'd': [0.4, -0.4, 0.4, -0.2, 0.4, 0.1, 0.4, 0.2, 0.4, 0.1, 0.4, 0.2],  # volar
        'e': [0.3, -0.3, 0.3, -0.1, -0.7, 0.3, 0.9, 1.4, -0.3, 0.3, 0.9, 1.4],  # pelea
    }

    if pose not in poses:
        rospy.logerr("Pose {} no está definida".format(pose))
        return

    joint_goal = poses[pose]

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
    except MoveItCommanderException as e:
        rospy.logerr("Error setting joint target: {}".format(e))
        return

    # Actualiza el estado del robot
    move_group.set_start_state_to_current_state()

    # Aumenta la tolerancia permitida
    move_group.set_goal_joint_tolerance(0.1)
    move_group.set_goal_position_tolerance(0.1)
    move_group.set_goal_orientation_tolerance(0.1)

    # Planifica la trayectoria
    rospy.loginfo("Planning trajectory...")
    plan = move_group.plan()
    if not plan or not plan[1]:
        rospy.logerr("Planning failed")
        return

    # Publica la trayectoria planificada en el tópico /move_group/display_planned_path
    rospy.loginfo("Publishing planned trajectory...")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan[1])

    # Publica la trayectoria planificada en el tópico /move_group/display_planned_path
    display_trajectory_publisher.publish(display_trajectory)

    # Ejecuta la trayectoria planificada
    rospy.loginfo("Executing plan...")
    success = move_group.execute(plan[1], wait=True)
    if not success:
        rospy.logerr("Execution failed")
        return

    rospy.loginfo("Plan executed successfully")

    move_group.stop()  # Asegura que no haya movimiento residual

    # Cierra y limpia
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Uso: {} <pose>".format(sys.argv[0]))
        sys.exit(1)

    pose = sys.argv[1]
    try:
        move_group_python_interface(pose)
    except rospy.ROSInterruptException:
        pass