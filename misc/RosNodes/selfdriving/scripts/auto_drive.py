from __future__ import print_function

import pandas as pd
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import constants
from sensor_listener import SensorListener


class AutoDrive(SensorListener):
    def __init__(self):
        SensorListener.__init__(self)
        self._command_publisher = None

    def init(self):
        SensorListener.init(self)
        self._command_publisher = rospy.Publisher(constants.VELOCITY_ACTION_TOPIC, Twist, queue_size=10)

    def sonar_callback(self, data):
        SensorListener.sonar_callback(self, data)
        if self._command_publisher:
            left_val = self._sensor_values.get(constants.LEFT_SENSOR_KEY)
            right_val = self._sensor_values.get(constants.RIGHT_SENSOR_KEY)
            center_val = self._sensor_values.get(constants.CENTER_SENSOR_KEY)
            angular_z = int(10 * ((left_val - right_val) / (left_val + right_val))) / 10.0
            linear_x = 1
            cmd_vel = Twist(Vector3(linear_x, 0, 0), Vector3(0, 0, angular_z))
            print(cmd_vel)
            self._command_publisher.publish(cmd_vel)

if __name__ == '__main__':
    rosnode = AutoDrive()
    try:
        rosnode.init()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
