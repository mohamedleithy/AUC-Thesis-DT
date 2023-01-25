.PHONY:	turtlebot3_sim
turtlebot3_sim: 
	docker run -it --net=ros --env="DISPLAY=novnc:0.0" --env="HOST_IP=10.40.7.108" --env="ROS_MASTER_URI=http://roscore:11311" --env="TURTLEBOT3_MODEL=waffle_pi" turtlebot3_sim bash

.PHONY: ros_network
ros_network:
	docker network create ros

.PHONY: roscore_sim
roscore_sim:
	docker run -d --net=ros -p 11311:11311 --name roscore osrf/ros:noetic-desktop-full roscore

.PHONY: sensor_values
sensor_values:
	nohup python3 /AUC-Thesis-DT/ROS-master/cloudconnect/Reactive.py > output.log & && python3 ~/catkin_ws/src/sensor_fusion/src/sensor_fusion.py

.PHONY: stream
stream:
	nohup python3 /AUC-Thesis-DT/ROS-master/cloudconnect/Streaming.py > output.log & && python3 ~/catkin_ws/src/laser_values/src/scan.py

.PHONY: actuate
actuate:
	python3 ~/catkin_ws/src/actuate/src/actuate.py && nohup python3 /AUC-Thesis-DT/ROS-master/cloudconnect/TeleOperations.py > output.log & 
	

