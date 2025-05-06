# Application of Artificial Intelligence to a Humanoid Robot for Stress Treatment  
![Image of Yaren](https://github.com/RAMEL-ESPOL/YAREN/blob/main/YarenPerfil.png)

## System Execution
### Step 1  
Enable permissions  
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Step 2  
Launch the full system  
```bash
roslaunch yaren_u2d2 yaren_full.launch
```


## Adding Movements  
![Image of Yaren](https://github.com/RAMEL-ESPOL/YAREN/blob/main/InterfazMovimientos.png)

### Step 1  
Enable permissions  
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Step 2  
```bash
roslaunch yaren_u2d2 yaren_communication_datos.launch
```
Activate motor torque (see the image — section 1).  
Manually move the joints to a desired position.  
Then, print those positions (section 2) to save them into a `.txt` file.  
Save the `.txt` file and associate it in the config.  
Section 3 allows you to manually enter an array of positions, and section 4 is for defining routines (i.e., multiple positions).
