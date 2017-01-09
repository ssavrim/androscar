# Ros nodes using Docker

This section contains a set of nodes:
- balltracker:converts Compressed image from Android application to OpenCV image. It will then try to track a green ball.
- selfdriving: performs self-driving machine-learning based on sensors.

To do that:
- Install docker and docker-compose
- Then in a terminal:
    - export ROS_MASTER_URI=http://<ip_address_of_your_robot>:11311 (.i.e http://192.168.1.1:11311)
    - export ROS_IP=<ip_address_of_your_host> (.i.e 192.168.1.2)
    - xhost +
    - cd balltracker (or cd selfdriving depending on what you want to do)
    - docker-compose build
    - docker-compose up

Notes:
- To start android embedded application remotely, you configure the device as adb over tcp:
    - adb tcpip 5555
    - adb connect <ip_address_of_your_robot> (.i.e 192.168.1.1)
    - To stop the application:
        adb shell am force-stop com.kreolite.androvision
    - To start the application:
        adb shell am start -n com.kreolite.androvision/.RosCameraActivity --ei cameraId 0

# Reference:

http://www.pyimagesearch.com/2015/09/21/opencv-track-object-movement/
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
