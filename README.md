# ros-noetic-rover-design



### ROS Libraries for Arduino
```bash
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
```

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





## 🧩 ROS Side — Install Web Tools

#### Install:
```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-web-video-server
```
