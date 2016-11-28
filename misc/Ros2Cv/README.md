This node aims at converting Compressed image from Android application to Opecn image.
It will then try to track a green ball.

To do that:
- Install docker then pull osrf/ros:indigo-desktop-full images
- Launch this image
- docker run -it -P -p11311:11311 --env="ROS_IP=172.17.0.2" --env="ROS_MASTER_URI=http://ip_of_master:11311" --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" osrf/ros:indigo-desktop-full bash
- rosrun image_transport republish compressed in:=/camera/image _image_t_transport:=compressed out:=camera2
- python balltracker.py

Reference:

http://www.pyimagesearch.com/2015/09/21/opencv-track-object-movement/
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython


TODO:
Make a docker file
