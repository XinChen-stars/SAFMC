# TODO
1.add manage final land position
2.add px4 auto land function

# SAFMC_tutorial
This repo is a ros workspace, containing SAFMC rospkgs:
* `gazebo` contains the gazebo model and world, and the launch files
* `controller` contains the px4 basic controller
* `simtools` contains the gazebo real pose launch files
* `apriltag_ros` contains the apriltag detection launch files

**Note:** If you are working on this project, it is necessary to intall [XTDrone](https://github.com/robin-shaun/XTDrone) first !



## Requirement
```bash
sudo apt-get install libeigen3-dev       
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```
### 0. Apriltag
```bash
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag/
mkdir build
cd build
cmake ..
make
sudo make install
```

### 1. SAFMC
```bash
cd ~/
mkdir -p SAFMC/src
cd ~/SAFMC/src
git clone https://github.com/XinChen-stars/SAFMC.git
cd ..
catkin_make
```

### 2. Configure Gazebo
```bash
cd ~/SAFMC/src/SAFMC/gazebo
cp -r ./gazebo_models/* ~/PX4_Firmware/Tools/sitl_gazebo/models
cp ./gazebo_worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds
cp ./gazebo_launch/search_single_simulation.launch ~/PX4_Firmware/launch
```

### 3. Configure PX4
```bash
rm -rf ~/PX4_Firmware/build/px4_sitl_default/etc/init.d-posix/rcS
cp ~/SAFMC/src/SAFMC/PX4_EKF/rcS ~/PX4_Firmware/build/px4_sitl_default/etc/init.d-posix/
rm ~/.ros/eeprom/parameters*
rm -rf ~/.ros/sitl*
```

## Usage

### 0. Gazebo
```bash
# Launch Gazebo World together with single drone
roslaunch px4 search_single_simulation.launch
# Launch Gazebo World together with swarm drone
roslaunch px4 search_swarm_simulation.launch
```

### 1. QGC
```bash
# Open QGC to monitor drone state
cd ~/your_QGC_path
./QGroundControl.AppImage
```

### 2. External Position
```bash
# Utilize the gazebo real pose
roslaunch simtools real_gazebo_pose.launch
```

### 3. PX4 Control
```bash
# Launch single drone px4 control
roslaunch px4_basic_ctrl px4_single_ctrl.launch
# Launch swarm drone px4 control
roslaunch px4_basic_ctrl px4_swarm_ctrl.launch
```

## 4. Apriltag detection
```bash
# Launch single drone apriltag detection
roslaunch apriltag_ros uav_detection_single.launch
# Launch swarm drone apriltag detection
roslaunch apriltag_ros uav_detection_swarm.launch
```



    





