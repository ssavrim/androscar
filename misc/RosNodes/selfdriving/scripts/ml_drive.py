from __future__ import print_function

import pandas as pd
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sklearn.externals import joblib

import constants
from sensor_listener import SensorListener


class PredictModel(SensorListener):
    def __init__(self, model_to_use=None, scaler_to_use=None):
        SensorListener.__init__(self)
        self._command_publisher = None
        # Load the model
	self._scaler = joblib.load(scaler_to_use)
        self._model = joblib.load(model_to_use)
        self._feature_columns = [constants.LEFT_SENSOR_KEY, constants.RIGHT_SENSOR_KEY]

    def init(self):
        SensorListener.init(self)
        # Init publisher to send predicted command
        self._command_publisher = rospy.Publisher(constants.VELOCITY_ACTION_TOPIC, Twist, queue_size=10)

    def X_scaling(self, scaler, values):
        # add dummy y value
        values.add(0.)
        scaled_values = scaler.transform(values)
        # remove y scaled value
        return scaled_values[:-1]

    def Y_inverse_scaling(self, scaler, value):
        # add dummy X values for left, right, delta
        values = [0., 0., 0.]  + value
        return scaler.inverse_transform(values)[-1]

    def sonar_callback(self, data):
        SensorListener.sonar_callback(self, data)
        if self._command_publisher:
            feature_values = [self._sensor_values.get(x) for x in self._feature_columns if x in self._sensor_values]
            # hack: the model was trained with the delta: left - right
            feature_values.append(self._sensor_values.get(constants.LEFT_SENSOR_KEY) - self._sensor_values.get(constants.RIGHT_SENSOR_KEY))
            feature_values = self.X_scaling(feature_values)
            predict_val = self._model.predict([feature_values])[0]
            predict_val = self.Y_inverse_scaling(predict_val)
            linear_x = 1 if predict_val != 0 else 0
            cmd_vel = Twist(Vector3(linear_x, 0, 0), Vector3(0, 0, predict_val))
            self._command_publisher.publish(cmd_vel)

if __name__ == '__main__':
    rosnode = PredictModel(model_to_use='model.pkl', scaler_to_use='scaler.pkl')
    try:
        rosnode.init()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

