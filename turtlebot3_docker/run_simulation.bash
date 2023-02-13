# create docker network 



docker run -d --rm --net=ros \
   --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" \
   --name=novnc -p=8080:8080 \
   theasp/novnc:latest 
docker run -d --net=ros -p 11311:11311 --name roscore osrf/ros:noetic-desktop-full roscore



docker run -it --net=ros --env="DISPLAY=novnc:0.0" --env="HOST_IP=10.40.7.108" --env="ROS_MASTER_URI=http://roscore:11311" --env="TURTLEBOT3_MODEL=waffle_pi" turtlebot3-noetic bash
source ~/catkin_ws/devel/setup.sh
roslaunch turtlebot3_gazebo turtlebot3_house.launch