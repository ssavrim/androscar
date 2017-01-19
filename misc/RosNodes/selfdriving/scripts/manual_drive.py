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
        self._linear_x = 0.0
        self._angular_z = 0.0

    def init(self):
        pygame.display.init()
        pygame.display.set_mode((320, 240))
        # Init publisher to send predicted command
        rospy.init_node(self.NODE_NAME, anonymous=True)
        self._command_publisher = rospy.Publisher(constants.VELOCITY_ACTION_TOPIC, Twist, queue_size=10)

    def start(self):
        if self._command_publisher:
            while not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        keys=pygame.key.get_pressed()
                        if keys[pygame.K_LEFT]:
                            print('LEFT')
                            self._angular_z += 0.1
                            self._angular_z  = min(self._angular_z, 1.0)
                        if keys[pygame.K_RIGHT]:
                            print('RIGHT')
                            self._angular_z -= 0.1
                            self._angular_z  = max(self._angular_z, -1.0)
                        if keys[pygame.K_UP]:
                            print('FORWARD')
                            self._linear_x = 1.0
                        if keys[pygame.K_DOWN]:
                            print('BACKWARD')
                            self._linear_x = -1.0
                        if keys[pygame.K_SPACE]:
                            print('STOP')
                            self._linear_x = 0.0
                            self._angular_z = 0.0
                        cmd_vel = Twist(Vector3(self._linear_x, 0, 0), Vector3(0, 0, self._angular_z))
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
