#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('cv_bridge')
import sys
import rospy
import cv2
import numpy as np
import imutils
from collections import deque
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
BUFFER_SIZE = 20


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera2",Image,self.callback)
        self.command_pub = rospy.Publisher("car_command/cmd_vel", Twist, queue_size=10)
        self._dX = 0
        self._dY = 0
        self._counter = 0
        self._direction = ""
        self._pts = deque(maxlen=BUFFER_SIZE)

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
        self.process_image(cv_image)

    def process_image(self, frame):
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # update the points queue
                self._pts.appendleft(center)

        # loop over the set of tracked points
        for i in xrange(1, len(self._pts)):
            # if either of the tracked points are None, ignore
            # them
            if self._pts[i - 1] is None or self._pts[i] is None:
                continue

            # check to see if enough points have been accumulated in
            # the buffer
            if self._counter >= 10 and i == 10 and self._pts[i-10] is not None:
                # compute the difference between the x and y
                # coordinates and re-initialize the direction
                # text variables
                self._dX = self._pts[i-10][0] - self._pts[i][0]
                angular_z = self._dX / (self._pts[i-10][0] + self._pts[i][0])
                self._dY = self._pts[i-10][1] - self._pts[i][1]
                linear_x = self._dY / (self._pts[i-10][1] + self._pts[i][1])
                (dirX, dirY) = ("", "")

                # ensure there is significant movement in the
                # x-direction
                if np.abs(self._dX) > 20:
                    dirX = "East" if np.sign(self._dX) == 1 else "West"

                # ensure there is significant movement in the
                # y-direction
                if np.abs(self._dY) > 20:
                    dirY = "South" if np.sign(self._dY) == 1 else "North"

                # handle when both directions are non-empty
                if dirX != "" and dirY != "":
                    self._direction = "{}-{}".format(dirY, dirX)

                # otherwise, only one direction is non-empty
                else:
                    self._direction = dirX if dirX != "" else dirY

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(BUFFER_SIZE / float(i + 1)) * 2.5)
            cv2.line(frame, self._pts[i - 1], self._pts[i], (0, 0, 255), thickness)

        # show the movement deltas and the direction of movement on
        # the frame
        cv2.putText(frame, self._direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 255), 3)
        cv2.putText(frame, "dX: {}, dY: {}".format(self._dX, self._dY),
                    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (0, 0, 255), 1)


        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        self._counter += 1

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
       rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
