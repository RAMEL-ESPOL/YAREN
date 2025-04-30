### Package Description

This package contains the necessary files for staging the robot, covering both the control of its movements and the visualization of its face.

### Package Structure

```
- launch/
  └── gazebo_simulation.launch
- movements/
- rostro/
- scripts/
  ├── movement.py
  ├── voz_movement.py
  ├── yaren_communication.py
  └── yaren_face.py
```

### File Descriptions

- **`movement.py`**: Script that allows control of the robot's positions through console commands.  
- **`voz_movement.py`**: Script that enables the execution of robot movements via voice commands. To operate correctly, it requires the [AI-FaceSoftware](https://github.com/RAMEL-ESPOL/AI-FaceSoftware.git) package. Some of the movements used in this script are read from `.txt` files located in the `movements/` folder, which have been previously generated using the movement creation interface.
- **`yaren_communication.py`**: Script used in **`gazebo_simulation.launch`**.
- **`yaren_face.py`**: Script that continuously loops a sequence of images representing the robot’s face. The images used are located in the `rostro/` folder.

### How to Add New Movements Using Generated `.txt` Files

To add new movements to the `voz_movement.py` file, go to the end section of the script, where the movements are defined by command number. You can follow the structure below using an `elif` block:

```python
elif self.number == '7':
    while not rospy.is_shutdown():
        file_path = movements_folder + "/baile_de_barney.txt"
        self.process_file(file_path)
```

Each `self.number` represents a command identifier associated with a specific movement. To add a new one, simply continue the numbering and specify the corresponding `.txt` file name:

```python
elif self.number == '8':
    while not rospy.is_shutdown():
        file_path = movements_folder + "/nuevo_baile.txt"
        self.process_file(file_path)
```

Ensure that the `.txt` file is located in the `movements/` folder and has been created using the movement interface.

