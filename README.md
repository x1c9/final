## Python libraries
```bash
pip install -r requirements.txt
```

## ROS packages
```bash
sudo apt-get install ros-noetic-amcl ros-noetic-explore-lite ros-noetic-gmapping ros-noetic-move-base ros-noetic-dwa-local-planner
```  

## Simulation in Gazebo
```bash
roslaunch final gazebo.launch
```  

## Simulation in rviz
```bash
roslaunch final display.launch
```

## Autonomous map scanning
```bash
roslaunch final auto_explore.launch
```

## Nagivation manually through 2d navigation arrow
```bash
roslaunch final move_base.launch
```
