# TODO
- Add EKF fuction to improve the precision of the apriltag detection
- Add the apriltag detection to the exploration fsm

# Apriltag_tutorial
## 1.USB-CAM
### (1)安装USB-CAM功能包
    sudo apt install ros-noetic-usb-cam
### (2)修改USB相关参数：
    roscd usb_cam/launch/
    sudo vim usb_cam-test.launch
        <param name="video_device" value="/dev/video0" />
        <param name="image_width" value="640" />
        <param name="image_height" value="480" />
        <param name="pixel_format" value="mjpeg" />
### (3)启动USB相机：
    roslaunch usb_cam usb_cam-test.launch 

## 2.棋盘格标定包
### (1)安装棋盘格标定包
    sudo apt-get install ros-noetic-camera-calibration
    棋盘格下载链接：http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf
### (2)开始标定
    roslaunch usb_cam usb_cam-test.launch
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam
    参数说明：
    size：    棋盘内交叉点的个数，行*列
    square：  一个格子的边长，单位是m
    image：  订阅摄像头发布的图像话题（ROS topic）
    camera： 寻找相应的设备相机名
    技巧说明：
    相机不运动，移动标定板，让4条标定轴达到绿色，点击calibrate计算出结果，然后点击save、commit,标定文件会自动保存在/tmp路径下，文件名为/tmp/calibrationdata.tar.gz
    ~/.ros/camera_info/head_camera.yaml,roslaunch usb_cam usb_cam-test.launch会自动读取相机内参文件

## 3.编译安装依赖库apriltag
    git clone https://github.com/AprilRobotics/apriltag.git
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

## 4.安装apriltag_ros功能包
    cd apriltag_ws/src
    git clone https://github.com/AprilRobotics/apriltag_ros.git
    cd ..
    catkin_make

## 5.下载预先生成的apriltag图片
### (1)进入网站下载    
    https://github.com/AprilRobotics/apriltag-imgs
### (2)选择tag_family
    cd /tagCustom48h12
    sudo apt-get install ghostscript
    ps2pdf alltags.ps alltags.pdf
### (3)后期处理

## 6.apriltag_ros配置
### (1)修改apriltag_ros/apriltag_ros/config/settings.yaml
    tag_family:        'tagCustom48h12' 
    tag_threads:       2          
    tag_decimate:      1.0       
    tag_blur:          0.0       
    tag_refine_edges:  1        
    tag_debug:         0         
    max_hamming_dist:  2          
    publish_tf:        true      
    transport_hint:    "raw"      
### (2)修改apriltag_ros/apriltag_ros/config/tags.yaml
    standalone_tags:
    [
        {id: 0, size: 0.12},
        {id: 1, size: 0.024},
        {id: 2, size: 0.005},
    ]
    tag_bundles:
    [
        {
        name: 'cx_tag_bundle',
        layout:
            [
            {id: 0, size: 0.12, x: 0.0000, y: 0.0000, z: 0.0000, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0},
            {id: 1, size: 0.024, x: 0.0000, y: 0.0000, z: 0.0000, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0},
            {id: 2, size: 0.005, x: 0.0000, y: 0.0000, z: 0.0000, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0}
            ]
        }
    ]
### (3)修改apriltag_ros/apriltag_ros/launch/continuous_detection.launch
    <arg name="camera_name" default="/usb_cam" />
    <arg name="image_topic" default="image_raw" />
    <arg name="queue_size" default="1" />

## 7.启动apriltag检测
    roslaunch apriltag_ros continuous_detection.launch
    rostopic echo /tag_detections







