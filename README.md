# ROS1 Jessiarm

## RobotArm with ROS Molodic + Donkeycar!!

로드밸런스팀 김수영씨 아래 코드를 가져와서 수정해서 만들고 있습니다.

https://github.com/Road-Balance/donkey_ros

<p align="center">
    <img src="./Images/joy_control.gif" width="400" />
    <img src="./Images/keyboard_control.gif" width="400" />
    <img src="./Images/blob_tracking.gif" width="400" />
    <img src="./Images/yolo_control.gif" width="400" />
</p>

There's Notion Lecture Notes and Youtube video's about this project. 
But, It's written in Korean. Anyway, Here's the link

* [Notion Lecture Notes] https://www.notion.so/JessiArm-be431f54912b472fb7f8977e5499612d


## Tested System information

**Jetson Nano 2GB + USB camera**

* Ubuntu 18.04
* ROS Melodic
* Opencv3.4.6

## Packages with Brief Explanation

```
├── csi_camera => Handling Image data for USB camera 
├── donkey_control => Control RC Car with Adafruit PCA9685
├── donkey_cv => Computer Vision Package with Opencv3.4.6
├── donkey_joy => Control RC Car with Gamepad 
│
(...)
├── Images
├── LICENSE
├── README.md
```

## Prerequisite

1. Ros Packages installation
   
```bash
$ sudo apt-get install ros-melodic-cv-bridge
$ sudo apt-get install ros-melodic-image-view
```

2. Clone this Repo

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/zeta0707/donkey_ros_arm.git

$ cd ../
$ catkin_make
$ source devel/setup.bash
```

## Usage

1. csi_camera package

Packages for Image Streaming

> Check Camera Connection First!!!

```bash
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
   'video/x-raw(memory:NVMM),width=3280, height=2464, framerate=21/1, format=NV12' ! \
   nvvidconv flip-method=2 ! 'video/x-raw,width=960, height=720' ! \
   nvvidconv ! nvegltransform ! nveglglessink -e
```

* `sensor_id` : this value depends on Camera Slot in Jetson Nano.

### Webcam Puslish

```bash
$ roscore

$ rosrun csi_camera webcam_pub.py
$ rosrun image_view image_view image:=/csi_image
```

2. donkey_control package

Packages for controlling `RC Car` with `PCA9685` PWM driver.
You need to install `Adafruit_PCA9685` python package first 

There's four modes for controlling RC Car

* JoyStick Control
* Keyboard Control
* Blob Control
* Yolo4 Control

```bash
$ roscore

$ rosrun donkey_control joy_control.py
$ rosrun donkey_control blob_chase.py
```

3. donkey_joy package

There's two modes for using joystick

* Axes mode

```bash
$ roslaunch donkey_joy joy_teleop_axes.launch
```

4. donkey_cv package

Packages for OpenCV applications

* Find Blob with Certain color
* Publish Image location as a `geometry_msgs/Point`

```bash
$ roscore

$ rosrun donkey_cv find_ball.py
```

## Application

### **1. joy_control**

Control RC Car with game controller

<p align="center">
    <img src="./Images/joy_control.gif" width="500" />
</p>

```bash
$ roscore

# Jetson
$ rosrun donkey_control joy_control.py

# Laptop or Jetson
$ roslaunch donkey_joy joy_teleop_axes.launch


왼쪽 레버 좌우: motor0 회전
왼쪽 레버 상하: motor1 기울어짐
오른쪽 레버 좌우: gripper 회전
오른쪽 레버 상하: motor2,3 기울어짐
왼쪽 조그셔틀 상하: Gripper open/close
```

### **2. keyboard_control**

Control RC Car with keyboard

<p align="center">
    <img src="./Images/keyboard_control.gif" width="500" />
</p>

```bash
$ roscore

# Jetson
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Laptop or Jetson
$ roslaunch donkey_joy joy_teleop_axes.launch

j l: 로봇암 좌우
i , : motor0 기울어짐
o . : motor2,3 기울어짐
u m : gripper 회전
t b: gripper 열림, 닫힘
```

### **3. blob_tracking**

Find the any color box of the Jetson Nano on the screen and change the direction of the wheel accordingly.


<p align="center">
    <img src="./Images/blob_tracking.gif" width="500" />
</p>


```bash
$ roscore

$ rosrun csi_camera csi_pub.py
$ rosrun donkey_cv find_ball.py 
$ rosrun donkey_control chase_the_ball.py 
$ rosrun donkey_control blob_chase.py 
```

Debugging with `image_view`

```bash
rosrun image_view image_view image:=/webcam_image
rosrun image_view image_view image:=/blob/image_mask
rosrun image_view image_view image:=/blob/image_blob
```

### **4. Yolo4_tracking**

Find the object of the Jetson Nano on the screen and change the direction of the wheel accordingly.


<p align="center">
    <img src="./Images/yolo_control.gif" width="500" />
</p>


```bash
#terminal #1
#object detect using Yolo_v4
zeta@zeta-nano:~/catkin_ws$ roslaunch darknet_ros yolo_v4.launch

#terminal #2
zeta@zeta-nano:~/catkin_ws$ roslaunch donkey_control yolo_chase.launch
```
