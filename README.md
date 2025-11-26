# ros-noetic-rover-design





#

## 🚀 2. PREPARE ENVIRONMENT (Pi & PC)
Install build tools and dependencies.

### On the Pi (Raspberry Pi OS 64-bit or Ubuntu 20.04 ARM64)
```bash
sudo apt update
sudo apt install -y build-essential python3-catkin-tools python3-rosdep python3-vcstool cmake git
```
### On the PC (Ubuntu 20.04)
```bash
sudo apt update
sudo apt install -y build-essential python3-catkin-tools python3-rosdep python3-vcstool cmake git
```

## 🚀 3. Install ROS (if not installed)

### On the Pi (Noetic Ubuntu 20.04 ARM64)
```bash
sudo apt install -y ros-noetic-desktop-full
```
### On the PC (Noetic Ubuntu 20.04)
```bash
sudo apt install -y ros-noetic-ros-base
or
sudo apt install -y ros-noetic-robot
```



sudo apt install ros-noetic-rosserial ros-noetic-rosserial-python

sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control



#

### ROS Libraries for Arduino
```bash
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
```



## 🧩 ROS Side — Install Web Tools

#### Install:
```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-web-video-server
```
