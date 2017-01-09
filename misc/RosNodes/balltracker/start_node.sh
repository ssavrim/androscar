#!/bin/bash

adb shell am force-stop com.kreolite.androvision
adb shell am start -n com.kreolite.androvision/.RosCameraActivity --ei cameraId 1
docker-compose up
