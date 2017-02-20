from __future__ import print_function

import pandas as pd
import numpy as np
import rospy
import pygame
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import constants


class ManualDrive(object):
    NODE_NAME = "manual_drive"

    def __init__(self):
        self._command_publisher = None
        self._throttle = 0.0
        self._steering = 0.0

    def init(self):
        pygame.display.init()
        pygame.display.set_mode((640, 480))
        pygame.key.set_repeat(500, 30)
        rospy.init_node(self.NODE_NAME, anonymous=True)
        self._command_publisher = rospy.Publisher(constants.VELOCITY_ACTION_TOPIC, Twist, queue_size=10)

    def start(self):
        if self._command_publisher:
            while not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        keys=pygame.key.get_pressed()
                        if keys[pygame.K_LEFT]:
                            print("Go left")
                            self._steering += 0.1
                            self._steering = min(self._steering, 1.0)
                        if keys[pygame.K_RIGHT]:
                            print("Go right")
                            self._steering -= 0.1
                            self._steering = max(self._steering, -1.0)
                        if keys[pygame.K_UP]:
                            print("Go forward")
                            self._throttle = 1.0
                            self._steering = 0.0
                        if keys[pygame.K_DOWN]:
                            print("Go backward")
                            self._throttle = -1.0
                            self._steering = 0.0
                        if keys[pygame.K_SPACE]:
                            print("Stop")
                            self._throttle = 0.0
                            self._steering = 0.0
                        cmd_vel = Twist(Vector3(self._throttle, 0, 0), Vector3(0, 0, self._steering))
                        self._command_publisher.publish(cmd_vel)

        else:
            print("Warning ! Command publisher is not initialized !")

if __name__ == '__main__':
    rosnode = ManualDrive()
    try:
        rosnode.init()
        rosnode.start()
    except KeyboardInterrupt:
        print("Shutting down")
