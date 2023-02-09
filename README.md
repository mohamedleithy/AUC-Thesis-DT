# AUC-Thesis-DT
A Digital Twin Prototype of an Automated Guided Vehicle (Turtlebot3) 🐢. 

## Usage 

[Get **Docker**](https://docs.docker.com/get-docker/), set it up nicely, then, in your terminal:

### 1. Build the Docker Image to run simulation. 
````
cd ./turtlebot3_docker 
docker build -t turtlebot3-noetic .
````

### 2. Create a Docker Network for all containers to communicate through. 
````
docker network create ros
````
### 3. Pull theasp/novnc image and run the container. 
````
docker pull theasp/novnc:latest
docker run -d --rm --net=ros \
   --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" \
   --name=novnc -p=8080:8080 \
   theasp/novnc:latest
````
📺 on localhost:8080/vnc.html .
### 4. Launch a container running roscore. 

````
docker run -d --net=ros -p 11311:11311 --name roscore osrf/ros:noetic-desktop-full roscore
````
### 5. Run a container from the built image and replace <machine-ip>. 

````
docker run -it --net=ros --env="DISPLAY=novnc:0.0" --env="HOST_IP=<machine-ip>" \
   --env="ROS_MASTER_URI=http://roscore:11311" --env="TURTLEBOT3_MODEL=waffle_pi" \ 
   turtlebot3-noetic bash
````

### 6. Launch the turtlebot3 simulation. 💣
````
source ~/catkin_ws/devel/setup.sh
roslaunch turtlebot3_gazebo turtlebot3_house.launch
````

### 7. Launch the dashboard to monitor/actuate the turtlebot. 
````
pip3 install -r ./RemoteDrivingDashboard-master/requirements.txt
python3 ./OTA_RemoteDrivingConfigurator-main/Designs/QtGUI.py
````
then configure your dashboard. 🕹

### 8. On a new terminal window, run another container (step 3) and: 
#### a. Configure Aws for Kinesis Data & Video Streaming [check for help](https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-files.html) then: 

````
bash /AUC-Thesis-DT/RemoteDrivingDashboard-master/cloudconnect.sh
````

### 9. Activate any of the following nodes in a new container (step3): 


#### Actuate Node 
````
python3 ~/catkin_ws/src/actuate/src/actuate.py
````
#### Sensors Node 
````
python3 ~/catkin_ws/src/sensor_fusion/src/sensor_fusion.py
````
#### Streaming Node
````
python3 ~/catkin_ws/src/laser_values/src/scan.py
````

and have fun. 🎉

