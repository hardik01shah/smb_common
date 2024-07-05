# Setup
## 1. Copy files from robot
1. Set the host in `~/.ssh/config`, e.g.:
```txt
# mostly used for executing commands on SMB
Host smbnuc
  Hostname 10.0.4.5
  User team7

# may be used for object detection task on SMB
Host smb-264-gpu
  Hostname 10.0.4.7
  User team7
```

2. Create a script `copy_smbnuc.sh` in the smb workspace. And allow this file to be executed as a program in property settings.
> Note:
> 1. file name: object_detection.csv & map.pcd
> 2. path: remember to set your own username and paths.
```bash
# copy_smbnuc.sh
scp smbnuc:/home/<username>/object_detection.csv /<path_to_your_smb_workspace>/src/core/smb_result_visualization/object_visualizer/data/
scp smbnuc:/home/<username>/smb_ws/src/core/smb_slam/data/maps/map.pcd /<path_to_your_smb_workspace>/src/core/smb_result_visualization/pcd_visualizer/data/
```



3. Connect to the robot's WiFi and copy the `.csv` file and the `.pcd` file from the robot.
```bash
# cd to the smb workspace
./copy_smbnuc.sh
```

## 2. Visualize the map and the objects
1. Create a script `visualize_results.sh` in the smb workspace. And allow this file to be executed as a program in property settings.
```bash
#!/bin/bash

# Launch pcd
gnome-terminal -- bash -c "roslaunch pcd_visualizer view_pcd.launch; exec bash"
sleep 0.5  # Wait for 0.5 seconds

# Launch smb_msf_graph
gnome-terminal -- bash -c "roslaunch object_visualizer view_objects.launch; exec bash"
sleep 0.5  # Wait for 2 seconds
```
The clustered result `clustered.csv` can be found in `object_visualizer/data` 
## 3. Usage:
Connect to the smb's WiFi and cd to the smb workspace.
```bash
# In terminal 1
./copy_smbnuc.sh
```

```bash
# In terminal 2
./visualize_results.sh
```
Now you can see the map and the objects visualized in Rviz.