This folder contains all information in case the car embeds a raspberrypi connected to an arduino.

------------------       ----------------\n
|                |       |              |----reads---- ultrasounds sensors\n
|  RASPBERRY PI  | ------| ARDUINO NANO |\n
|  ROS+rosserial |       |   ros_lib    |----control-- motor\n
|                |       |              |\n
------------------       ----------------\n

# Prerequisite

Raspberry Pi (tested on RPI B+)
- ROS indigo, rosserial, rosserial_arduino
- arduino (sudo apt-get install arduino)
- picocom (sudo apt-get install picocom)
- ino build (sudo pip install ino)

Arduino (tested on arduino nano)
- ros_lib library
- NewPing library

# Setup and Installation
- ssh to raspberrypi
- git clone https://github.com/ssavrim/androvision.git
- cd androvision/misc/RaspberryPi
- copy ros_lib and NewPing libraries in ./lib folder
- ino clean
- ino build -m nano328
- ino upload -m nano328
- roslaunch serial_node.launch

# Reference

http://inotool.org/#installation
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
https://github.com/dreamster/rosserial-example/blob/master/rosserial-example.ino
http://wiki.ros.org/rosserial/Overview
http://playground.arduino.cc/Code/NewPing#Download

