This folder contains all information in case the car embeds a raspberrypi connected to an arduino.

------------------       ----------------
|                |       |              |----reads---- ultrasounds sensors
|  RASPBERRY PI  | ------| ARDUINO NANO |
|  ROS+rosserial |       |   ros_lib    |----control-- motor
|                |       |              |
------------------       ----------------

# Prerequisite

Raspberry Pi (tested on RPI B+)
- ROS indigo, rosserial, rosserial_arduino
- arduino (sudo apt-get install arduino)
- ino build (sudo pip install ino)

Arduino (tested on arduino nano)
- ros_lib library
- NewPing library

# Setup and Installation
- ssh to raspberrypi
- git clone https://github.com/ssavrim/androvision.git
- cd androvision/misc/RaspberryPi
- copy ros_lib and NewPing libraries in ./lib folder
- start.sh

# Reference

http://inotool.org/#installation
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
https://github.com/dreamster/rosserial-example/blob/master/rosserial-example.ino
http://wiki.ros.org/rosserial/Overview
http://playground.arduino.cc/Code/NewPing#Download

