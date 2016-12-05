#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cv_bridge')
import sys
import rospy
from sensor_msgs.msg import Range

class sensor_listener:

    def __init__(self):
        self.sonar_sub = rospy.Subscriber("car_sensor/front_range",Range,self.callback)

    def callback(self,data):
        print(data)

def main(args):
    ic = sensor_listener()
    rospy.init_node('sensor_listener', anonymous=True)
    try:
       rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
