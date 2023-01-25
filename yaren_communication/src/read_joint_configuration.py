import os
import time

from motor_classes import *
from dynamixel_sdk import * 

if __name__ == '__main__':
  usb_port =
  dxl_baud_rate = 
  
  num_joints = 12
  general_joint_position = [0 for i in range(num_joints)]
  general_joint_position_state = [0 for i in range(num_joints)]
    
  portHandler = PortHandler(usb_port)
  packetHandler = PacketHandler(2.0)
    
    
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
  
  time.sleep(5)
  
   for motor in list_motors:
    motor.torque(motor.torque_disable, motor.addr_torque_enable)
  
  # Torque on to fix the actual configuration
  
  actual_id = 100
  while actual_id != 0:
    actual_id = int(input("Indicate the motor´s id from which activated torque is required:"))
    if actual_id != 0:
      for motor in list_motors:
        if actual_id in motor.list_ids:
          motor.torque(motor.torque_enable, motor.addr_torque_enable)
   
  joint_values = [0,0,0,0,0,0,0,0,0,0,0,0]
  
  #Record and process the actual joint configuration
  for motor in list_motors:
    for id in motor.list_ids:
      dxl_present_position, dxl_comm_result, dxl_error = motor.packetHandler.read4ByteTxRx(motor.portHandler, id, motor.addr_present_position)
        if dxl_comm_result != COMM_SUCCESS:
          print("%s" % motor.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
          print("%s" % motor.packetHandler.getRxPacketError(dxl_error))
      new_value = motor.angleConversion(dxl_present_position,True,id)
      joint_values[id-1]
   print(joint_values) 
   time.sleep(5)
      
  
  #Torque off, end of code
  for motor in list_motors:
    motor.torque(motor.torque_disable, motor.addr_torque_enable)
