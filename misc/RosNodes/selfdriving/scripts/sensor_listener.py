import rospy
from sensor_msgs.msg import Range

import constants


class SensorListener(object):
    NODE_NAME = "sensor_listener"

    def __init__(self):
        self._sensor_values = {"/left": 0.0, "/center":0.0, "/right": 0.0}

    def init(self):
        rospy.init_node(self.NODE_NAME, anonymous=True)
        rospy.Subscriber(constants.FRONT_RANGE_TOPIC, Range, self.sonar_callback)

    def sonar_callback(self, data):
        if data.range == 0:
            return
        if data.range <= 10:
            rospy.logwarn("ALERT ! {} cm.".format(data.range))
        if data.header.frame_id in self._sensor_values:
            self._sensor_values[data.header.frame_id] = data.range
