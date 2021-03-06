
# 安装依赖
velodyne需要  `sudo apt install ros-${ROS_DISTRO}-velodyne`  
urg需要  `sudo apt install ros-${ROS_DISTRO}-urg-node`  
okatech需要  `sudo apt install ros-${ROS_DISTRO}-turtlebot-teleop`  
imu需要  `sudo apt install ros-${ROS_DISTRO}-microstrain-mips`  


# 设置环境变量
```
# ROS
source /opt/ros/kinetic/setup.bash
source ~/ws_pch18/setup.bash
```


# 安装cartographer_ros
```
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
mkdir -p ~/ws_cartographer
cd ~/ws_cartographer
wstool init src
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
src/cartographer/scripts/install_proto3.sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make_isolated --install --use-ninja
```


# 固定设备号
写入以下内容到文件`/etc/udev/rules.d/95-kuaro.rules`  
```
# KUARO
KERNEL=="ttyUSB*", ATTRS{product}=="USB HS SERIAL CONVERTER", ATTRS{serial}=="FTY1GCEQ", SYMLINK+="sensors/ttyUSBOkatech"
# Gyro
KERNEL=="ttyUSB*", ATTRS{product}=="USB HS SERIAL CONVERTER", ATTRS{serial}=="FT0B6GFC", SYMLINK+="sensors/ttyUSBGyro"
# GPS
KERNEL=="ttyUSB*", ATTRS{product}=="USB HS SERIAL CONVERTER", ATTRS{serial}=="FTXQOCXI", SYMLINK+="sensors/ttyUSBGPS"
```


# docker 下 CMD
```
bash -c '~/ws_pch18/setup.bash && bash'
```
