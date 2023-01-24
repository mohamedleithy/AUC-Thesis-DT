# create docker network 


docker run -d --net=ros -p 11311:11311 --name roscore osrf/ros:noetic-desktop-full roscore



docker run -it --net=ros --env="DISPLAY=novnc:0.0" --env="ROS_MASTER_URI=http://roscore:11311" --env="TURTLEBOT3_MODEL=waffle_pi" turtlebot3_sim bash
source ~/catkin_ws/devel/setup.sh

roslaunch turtlebot3_gazebo turtlebot3_house.launch