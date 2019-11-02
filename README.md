
velodyne需要`sudo apt install libpcap-dev`
urg需要 `sudo apt install ros-${ROS_DISTRO}-urg-node`
okatech需要 `sudo apt-get install ros-${ROS_DISTRO}-turtlebot-teleop`
serial_communication
`/etc/udev/rules.d/95-kuaro.rules`
```
# KUARO
KERNEL=="ttyUSB*", ATTRS{product}=="USB HS SERIAL CONVERTER", ATTRS{serial}=="FTY1GCEQ", SYMLINK+="sensors/ttyUSBOkatech"

# Gyro
KERNEL=="ttyUSB*", ATTRS{product}=="USB HS SERIAL CONVERTER", ATTRS{serial}=="FT0B6GFC", SYMLINK+="sensors/ttyUSBGyro"

# GPS
KERNEL=="ttyUSB*", ATTRS{product}=="USB HS SERIAL CONVERTER", ATTRS{serial}=="FTXQOCXI", SYMLINK+="sensors/ttyUSBGPS"

```