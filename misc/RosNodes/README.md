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
    - docker-compose build
    - docker-compose up

# Reference:

http://www.pyimagesearch.com/2015/09/21/opencv-track-object-movement/
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
