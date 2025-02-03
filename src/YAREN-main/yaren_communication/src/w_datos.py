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
import threading

import tkinter as tk
from tkinter import scrolledtext
from tkinter import messagebox
import ast

value_angles_max = []
value_angles_min = []

global_motors=[]

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

def torque_disable(list_motors):
    for motor in list_motors:
        for id in motor.list_ids:
            motor.torque(motor.torque_disable, motor.addr_torque_enable)

def torque_enable(list_motors):
    for motor in list_motors:
        for id in motor.list_ids:
            motor.torque(motor.torque_enable, motor.addr_torque_enable)

def get_positions(list_motors):# Read present position
    for motor in list_motors:
        for id in motor.list_ids:
            dxl_present_position, dxl_comm_result, dxl_error = motor.packetHandler.read4ByteTxRx(motor.portHandler, id, motor.addr_present_position)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % motor.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % motor.packetHandler.getRxPacketError(dxl_error))
            ##################
            #general_joint_position[id-1]= (dxl_present_position-2048) *0.088 
            general_joint_position[id-1]= round((dxl_present_position-2048) *0.088 * 3.14/180, 2)
            #general_joint_position[id-1]=motor.angleConversion(0,0,id)

def get_voltages(list_motors):
    for motor in list_motors:
        for id in motor.list_ids:
            # Leer voltaje (usualmente 2 bytes)
            dxl_present_voltage, dxl_comm_result, dxl_error = motor.packetHandler.read2ByteTxRx(
                motor.portHandler, id, motor.addr_present_voltage
            )
            
            if dxl_comm_result != COMM_SUCCESS:
                print("Error de comunicación: %s" % motor.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error del paquete: %s" % motor.packetHandler.getRxPacketError(dxl_error))
            else:
                # Ajustar según la escala de datos
                # Por ejemplo, si está en 0.1 V, dividir entre 10:
                dxl_present_voltage = dxl_present_voltage / 10.0
                
                # Guardar el voltaje procesado
                general_joint_voltage[id - 1] = dxl_present_voltage


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

def read_data_thread(list_motors):
    """
    Hilo que obtiene las posiciones y voltajes de los motores continuamente
    y guarda los datos en un archivo de texto.
    """
    # Abrir el archivo en modo de escritura (se sobrescribe cada vez que se ejecuta el programa)
    with open("/home/quadruped/Documents/Tesis_ESPOL_Reyes_Le-n_1/src/YAREN-main/yaren_master/data_log.txt", "w") as file:
        file.write("Tiempo,Posiciones,Voltajes\n")  # Escribe el encabezado del archivo

        while not rospy.is_shutdown():
            # Leer posiciones y voltajes
            get_positions(list_motors)
            get_voltages(list_motors)

            # Registrar la hora actual
            current_time = rospy.get_time()

            # Preparar datos como texto
            positions_text = ",".join(map(str, general_joint_position))
            voltages_text = ",".join(map(str, general_joint_voltage))

            # Escribir datos en el archivo
            file.write(f"{current_time},{positions_text},{voltages_text}\n")

            # También registrar en la consola
            rospy.loginfo("Posiciones: %s", positions_text)
            rospy.loginfo("Voltajes: %s", voltages_text)

            rospy.sleep(1/1000)  # Intervalo de 100 ms


if __name__ == '__main__':

    rospy.init_node("yaren_motor_data")

    r =rospy.Rate(10) # 10hz

    usb_port = rospy.get_param('~usb_port')
    dxl_baud_rate = rospy.get_param('~dxl_baud_rate')

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

    list_motors= [neck_motor,shoulder_right_motor,forearm_right_motor,elbow_right_motor,
    shoulder_left_motor,forearm_left_motor,elbow_left_motor,trunk_motor,hip_motor]

    
    #list_motors = [shoulder_right_motor,forearm_right_motor,forearm_right_motor]

    num_joints = 12
    general_joint_position = [0 for i in range(num_joints)]
    general_joint_position_state = [0 for i in range(num_joints)]


    general_joint_voltage = [0 for i in range(num_joints)]
    

    #Publish current robot state
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    set_positions({},[list_motors,True,num_joints,joint_state_pub])

    # Subscribe desired joint position
    rospy.Subscriber('/joint_goals', JointState,set_positions,(list_motors,False,num_joints,joint_state_pub), queue_size=5)
    print("subcribir")

        # Inicia el hilo para leer datos
    #read_thread = threading.Thread(target=read_data_thread, args=(list_motors,))
    #read_thread.daemon = True  # Daemon para que se cierre automáticamente al salir
    #read_thread.start()

    rospy.sleep(1)
    torque_disable(list_motors)

    # Intervalo de 100 ms
    # Crear un mensaje JointState
    data = JointState()
    #data.position = [0.26, -0.06, 0.05, 0.36, 1.42, 0.086, 0.07, -1.29476, -0.3809, 0.00307, 0.047, -0.3]
    #[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #data.position = [-0.43999999999999995, -1.8479999999999999, -0.7919999999999999, -7.4799999999999995, 13.815999999999999, -18.04, -6.864, -9.591999999999999, -17.336, -18.392, 1.408, -1.936]
 
    #set_positions(data,[list_motors,False,num_joints,joint_state_pub])

    
    # Función para actualizar el texto de posiciones
    def update_positions_display():
        positions_text.delete("1.0", tk.END)
        positions_text.insert(tk.END, str(general_joint_position))


    # Agregar atributos dinámicamente a los objetos de motor
    for motor, name in zip(list_motors, [
        "neck_motor", "shoulder_right_motor", "forearm_right_motor", "elbow_right_motor",
        "shoulder_left_motor", "forearm_left_motor", "elbow_left_motor", "trunk_motor", "hip_motor"
    ]):
        motor.name = name
        motor.torque_state = False  # Inicialmente desactivado

    # Función para actualizar el texto de posiciones
    def update_positions_display():
        positions_text.delete("1.0", tk.END)
        positions_text.insert(tk.END, str(general_joint_position))

    # Crear la ventana
    root = tk.Tk()
    root.title("Motor Torque Control")

    # Dividir la ventana en dos secciones
    frame_left = tk.Frame(root)
    frame_left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

    frame_right = tk.Frame(root)
    frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

    # Sección izquierda
    motor_buttons = {}

    def toggle_motor(motor, button):
        if not motor.torque_state:
            motor.torque(motor.torque_enable, motor.addr_torque_enable)
            motor.torque_state = True
            button.configure(bg="blue")
        else:
            motor.torque(motor.torque_disable, motor.addr_torque_enable)
            motor.torque_state = False
            button.configure(bg="grey")

    frame_buttons = tk.Frame(frame_left)
    frame_buttons.pack(pady=10)

    for motor in list_motors:
        button = tk.Button(
            frame_buttons, text=motor.name, bg="grey", width=20,
            command=lambda m=motor: toggle_motor(m, motor_buttons[m])
        )
        motor_buttons[motor] = button
        button.pack(pady=5)

    # Botón "All"
    def toggle_all():
        if all(not motor.torque_state for motor in list_motors):
            torque_enable(list_motors)
            for motor in list_motors:
                motor.torque_state = True
            for button in motor_buttons.values():
                button.configure(bg="blue")
        else:
            torque_disable(list_motors)
            for motor in list_motors:
                motor.torque_state = False
            for button in motor_buttons.values():
                button.configure(bg="grey")

    def print_positions():
        print("Get position")
        get_positions(list_motors)
        print(general_joint_position)
        update_positions_display()

    button_all = tk.Button(frame_left, text="All", bg="grey", width=20, command=toggle_all)
    button_all.pack(pady=5)

    positions_text = scrolledtext.ScrolledText(frame_left, width=50, height=5)
    positions_text.pack(fill=tk.X, padx=10, pady=5)
    positions_text.insert(tk.END, general_joint_position)

    button_print = tk.Button(frame_left, text="Print", command=print_positions)
    button_print.pack(pady=5)

    frame_set = tk.Frame(frame_left)
    frame_set.pack(pady=10)

    label_set = tk.Label(frame_set, text="Set Motor Positions")
    label_set.pack()

    entry_set = tk.Entry(frame_set, width=50)
    entry_set.pack(pady=5)

    label_message = tk.Label(frame_set, text="", fg="red")
    label_message.pack()

    def set_motor_positions():
        input_text = entry_set.get()
        try:
            # Convertir el texto ingresado a una lista
            positions = ast.literal_eval(input_text)

            # Validar que sea una lista de números
            if not isinstance(positions, list) or not all(isinstance(x, (int, float)) for x in positions):
                raise ValueError("El formato no es válido")

            # Crear un mensaje JointState y asignar posiciones
            data = JointState()
            data.position = positions

            # Llamar a la función set_positions
            set_positions(data, [list_motors, False, num_joints, joint_state_pub])

            # Limpiar el mensaje de error
            label_message.config(text="")
        except (ValueError, SyntaxError):
            label_message.config(text="El texto no coincide con el formato")

    button_set = tk.Button(frame_set, text="Set", command=set_motor_positions)
    button_set.pack(pady=5)



    # Sección derecha
    label_sequences = tk.Label(frame_right, text="Motion Sequences")
    label_sequences.pack()

    frame_sequences = tk.Frame(frame_right)
    frame_sequences.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    sequence_numbers = tk.Text(frame_sequences, width=4, height=20, state=tk.DISABLED, bg="#f0f0f0")
    sequence_numbers.pack(side=tk.LEFT, fill=tk.Y)

    sequence_text = tk.Text(frame_sequences, width=46, height=20)
    sequence_text.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    stop_execution = False  # Variable global para detener la ejecución

    def update_sequence_numbers(*args):
        lines = sequence_text.get("1.0", tk.END).strip().split("\n")
        sequence_numbers.config(state=tk.NORMAL)
        sequence_numbers.delete("1.0", tk.END)
        for i in range(1, len(lines) + 1):
            sequence_numbers.insert(tk.END, f"{i}.\n")
        sequence_numbers.config(state=tk.DISABLED)

    sequence_text.bind("<KeyRelease>", update_sequence_numbers)

    update_sequence_numbers()

    def execute_sequences():
        global stop_execution
        stop_execution = False
        lines = sequence_text.get("1.0", tk.END).strip().split("\n")

        for i, line in enumerate(lines):
            if stop_execution:
                print("Ejecución detenida")
                break

            try:
                # Separar la posición del tiempo
                parts = line.split(";")
                positions = ast.literal_eval(parts[0].strip())
                time = float(parts[1].strip()) if len(parts) > 1 else 1.0

                # Validar formato
                if not isinstance(positions, list) or not all(isinstance(x, (int, float)) for x in positions):
                    raise ValueError

                # Crear mensaje y ejecutar
                data = JointState()
                data.position = positions
                set_positions(data, [list_motors, False, num_joints, joint_state_pub])

                # Resaltar la línea actual
                sequence_text.tag_remove("highlight", "1.0", tk.END)  # Eliminar resaltado previo
                start_index = f"{i + 1}.0"
                end_index = f"{i + 1}.end"
                sequence_text.tag_add("highlight", start_index, end_index)
                sequence_text.tag_config("highlight", background="yellow")
                
                # Forzar actualización de la interfaz gráfica
                root.update()

                rospy.sleep(time)
            except Exception as e:
                print(f"Error ejecutando línea: {line}, {e}")

        # Eliminar resaltado al finalizar
        sequence_text.tag_remove("highlight", "1.0", tk.END)

    # Asigna la función al botón de ejecución
    button_execute = tk.Button(frame_right, text="Execute Sequences", command=lambda: threading.Thread(target=execute_sequences).start())
    button_execute.pack(pady=5)

    # Botón para detener la ejecución
    def stop_sequences():
        global stop_execution
        stop_execution = True

    button_stop = tk.Button(frame_right, text="Stop Execution", command=stop_sequences)
    button_stop.pack(pady=5)

    # Botón para cargar secuencias desde un archivo
    def load_sequences():
        try:
            file_path = tk.filedialog.askopenfilename(
                title="Select Sequence File",
                filetypes=[("Text Files", "*.txt")]
            )
            if file_path:
                with open(file_path, "r") as file:
                    sequence_text.delete("1.0", tk.END)
                    sequence_text.insert(tk.END, file.read())
                update_sequence_numbers()
        except Exception as e:
            print(f"Error loading sequences: {e}")

    button_load = tk.Button(frame_right, text="Load Sequences", command=load_sequences)
    button_load.pack(pady=5)

    root.mainloop()




    while not rospy.is_shutdown():  
        print("Antes del Spin")
        rospy.spin()
        print("Escuchando...")
        #joint_state_publisher(list_motors,num_joints,joint_state_pub)
        r.sleep()
    portHandler.closePort()



    



    
        



    
