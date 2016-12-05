#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cv_bridge')
import sys
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

_last_left = 0.0
_last_right = 0.0
_last_x = 0.0
_last_z = 0.0

def sonar_callback(data):
    global _last_left, _last_right, _last_x, _last_z
    if data.range == 0:
        return
    if data.header.frame_id == "/left":
        _last_left = data.range
    else:
        _last_right = data.range
    if _last_right <= 10 or _last_left <= 10 :
        rospy.logwarn("ALERT !")
    rospy.loginfo([_last_left, _last_right, _last_x, _last_z])


def command_callback(data):
    global _last_x, _last_z
    _last_x = data.linear.x
    _last_z = data.angular.z

def main(args):
    rospy.init_node('sensor_listener', anonymous=True)
    rospy.Subscriber("car_sensor/front_range", Range, sonar_callback)
    rospy.Subscriber("car_command/cmd_vel", Twist, command_callback)
    try:
       rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
