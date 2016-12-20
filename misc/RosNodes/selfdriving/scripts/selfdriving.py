from __future__ import print_function

import pandas as pd
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeRegressor

import constants
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

class PredictModel(SensorListener):
    def __init__(self, dataset_file, model_to_use=None):
        SensorListener.__init__(self)
        if model_to_use is None:
            model_to_use = DecisionTreeRegressor()
        self._model = model_to_use
        self._dataset_file = dataset_file
        self._command_publisher = None
        self._feature_columns = None
        # Build the model
        self.build_model()

    def build_model(self):
        # Build the model
        df = pd.read_csv(self._dataset_file, index_col=0)
        target_columns = constants.ANGULAR_KEY
        feature_data = df.drop(target_columns, axis=1)
        x = feature_data.values.astype(float)
        self._feature_columns = feature_data.columns
        y = df[target_columns].values.astype(float)
        x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)
        self._model.fit(x_train, y_train)
        print("Model accuracy: {}".format(self._model.score(x_test, y_test)))

    def init(self):
        SensorListener.init(self)
        # Init publisher to send predicted command
        self._command_publisher = rospy.Publisher(constants.VELOCITY_ACTION_TOPIC, Twist, queue_size=10)

    def sonar_callback(self, data):
        SensorListener.sonar_callback(self, data)
        if self._command_publisher:
            feature_values = [self._sensor_values.get(x) for x in self._feature_columns if x in self._sensor_values]
            predict_val = self._model.predict([feature_values])[0]
            linear_x = 1 if predict_val != 0 else 0
            cmd_vel = Twist(Vector3(linear_x, 0, 0), Vector3(0, 0, predict_val))
            self._command_publisher.publish(cmd_vel)

if __name__ == '__main__':
    model = RandomForestRegressor(n_estimators=250)
    rosnode = PredictModel(dataset_file=constants.DATASET_FILE, model_to_use=model)
    try:
        rosnode.init()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

