#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cv_bridge')
import sys
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import pandas as pd
from sklearn.tree import DecisionTreeRegressor
from sklearn.ensemble import RandomForestRegressor

MyDataFrame = pd.DataFrame(columns=["SonarLeft", "SonarRight", "AngularZ"])
MyModel = DecisionTreeRegressor()
DATASET_FILE = "/dataset/dataset.csv"
command_pub = None
command_mode = "train" # 'train' or 'predict'

_last_left = 0.0
_last_right = 0.0
_last_x = 0.0
_last_z = 0.0

def build_model():
    global MyModel
    df = pd.read_csv(DATASET_FILE, index_col=0)
    print(df.describe())
    target_columns = "AngularZ"
    x = df.drop(target_columns, axis=1).values.astype(float)
    y = df[target_columns].values.astype(float)
    MyModel.fit(x, y)


def sonar_callback(data):
    global _last_left, _last_right, _last_x, _last_z, command_mode, command_pub
    if data.range == 0:
        return
    if data.header.frame_id == "/left":
        _last_left = data.range
    else:
        _last_right = data.range
    if _last_right <= 10 or _last_left <= 10:
        rospy.logwarn("ALERT !")

    if command_mode == "predict" and command_pub:
        predict_val = MyModel.predict([[_last_left, _last_right]])[0]
        linear_x = 1 if predict_val != 0 else 0
        cmd_vel = Twist(Vector3(linear_x, 0, 0), Vector3(0, 0, predict_val))
        print("Predict value" + str(cmd_vel))
        command_pub.publish(cmd_vel)

def command_callback(data):
    global _last_x, _last_z, MyDataFrame
    _last_x = data.linear.x
    _last_z = data.angular.z
    MyDataFrame.loc[len(MyDataFrame)] = [_last_left, _last_right, _last_z]
    MyDataFrame.to_csv(DATASET_FILE)

def main(args):
    global MyModel, command_pub, command_mode
    if len(args) < 2:
        command_mode = "train"
    else:
        command_mode = args[1]

    print("Launching script in {0} mode.".format(command_mode))

    rospy.init_node('sensor_listener', anonymous=True)
    rospy.Subscriber("car_sensor/front_range", Range, sonar_callback)
    if command_mode == "predict":
        build_model()
        command_pub = rospy.Publisher("car_command/cmd_vel", Twist, queue_size=10)
    else:
        rospy.Subscriber("car_command/cmd_vel", Twist, command_callback)

    try:
       rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

