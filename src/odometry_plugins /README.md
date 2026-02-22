odometry_pugins
====
## Description
It is for ykc's odometory_board.
It works on ros2 Humble Hawksbill and Foxy Fitzroy. (Using Foxy is not recommended.)
Also, you should change new udev rule. See Install section.

This supports only environment as 1 byte of 8 bits. (uint8_t == unsigned char)


## Usage

you shold only launch launch/odometry_bridge_launch.xml .
you would get odometry raw data.

## Install

copy udev rule
```
sudo cp ~/ros2_ws/src/odometory_plugins/udev/60-odometry.rules /etc/udev/rules.d/60-odometry.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
