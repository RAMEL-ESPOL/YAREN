### Package Description

This package contains the necessary files for communication and control of the robot's motors, as well as for displaying the animated face through video playback. It integrates movement and visualization functions using other support packages, such as **AI-FaceSoftware**.

### Available Launch Files

This package includes two main `launch` files:

- **`yaren_full.launch`**:  
  Executes the full startup of the system.  
  - Starts communication with the motors using `yaren_communication.launch`.  
  - Sends movement commands to the robot.  
  - Plays the video of the robot's face by connecting to the [AI-FaceSoftware](https://github.com/RAMEL-ESPOL/AI-FaceSoftware.git) package. (Download the package first)

- **`yaren_communication_datos.launch`**:  
  Launches an interface that allows manual editing of motor positions.

### Package Structure

```
- launch/
  ├── yaren_communication_datos.launch
  ├── yaren_communication.launch
  └── yaren_full.launch    
- scripts/
  ├── u2d2_communication.py
  └── w_datos.py
```

### Script Descriptions

- **`u2d2_communication.py`**: Manages communication with the robot’s actuators through the U2D2 adapter.  
- **`w_datos.py`**: Script associated with the graphical interface for editing motor positions, launched by `yaren_communication_datos.launch`.
