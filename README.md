# TODO
- Add EKF fuction to improve the precision of the apriltag detection
- Add the apriltag detection to the exploration fsm

# SAFMC_tutorial
## 1.Install apriltag
    git clone https://github.com/AprilRobotics/apriltag.git
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

## 2.Install SAFMC ros pkg
    mkdir -p SAFMC/src
    cd SAFMC/src
    git clone https://github.com/XinChen-stars/SAFMC.git
    cd ..
    catkin_make

## 3.Configure gazebo environment
    cd SAFMC/src/SAFMC/gazebo
    cp ./gazebo_models/* ~/PX4_Firmware/Tools/sitl_gazebo/models
    cp ./gazebo_worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds
    cp ./gazebo_launch/search_single_simulation.launch ~/PX4_Firmware/launch

## 4.Start QGC to manual control the quadrotor
    ./QGroundControl.AppImage

## 5.Start PX4 Simulation
    roslaunch px4 search_single_simulation.launch
    roslaunch apriltag_ros uav_detection_single.launch







