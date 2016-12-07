from __future__ import print_function
import constants
import rospy
import pandas as pd
from geometry_msgs.msg import Twist
from sensor_listener import SensorListener


class TrainModel(SensorListener):
    def __init__(self, dataset_file):
        SensorListener.__init__(self)
        # TODO: include /linear.x value in the dataset.
        #self._sensor_values.update({LINEAR_KEY: 0.0, ANGULAR_KEY: 0.0})
        self._sensor_values.update({constants.ANGULAR_KEY: 0.0})
        self._dataset_file = dataset_file
        self._data_frame = pd.DataFrame(columns=sorted(self._sensor_values.keys()))

    def init(self):
        SensorListener.init(self)
        # Init listener to store commands from user during training phase
        rospy.Subscriber(constants.VELOCITY_ACTION_TOPIC, Twist, self.command_callback)

    def command_callback(self, data):
        # TODO: include /linear.x value in the dataset.
        #self._sensor_values[LINEAR_KEY] = data.linear.x
        self._sensor_values[constants.ANGULAR_KEY] = data.angular.z
        self._data_frame.loc[len(self._data_frame)] = [self._sensor_values[x] for x in sorted(self._sensor_values.keys())]
        self._data_frame.to_csv(self._dataset_file)

if __name__ == '__main__':
    rosnode = TrainModel(dataset_file=constants.DATASET_FILE)
    try:
        rosnode.init()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")