# AUC-Thesis-DT
A Digital Twin Prototype of an Automated Guided Vehicle (Turtlebot3) 🐢. 

## Usage 

[Get **docker and docker-compose**](https://docs.docker.com/get-docker/), set it up nicely, then, in your terminal:

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

### 5. Launch the server and DB by running: 

````
cd ../RemoteDrivingDashboard-master 
docker-compose up
````
 (on your localmachine not Dokcer). 

### 6. Run a container from the built image and replace machine-ip. 

````
docker run -it --net=ros --env="DISPLAY=novnc:0.0" --env="HOST_IP=<machine-ip>" \
   --env="ROS_MASTER_URI=http://roscore:11311" --env="TURTLEBOT3_MODEL=waffle_pi" \ 
   turtlebot3-noetic bash
````
### 7. Launch the dashboard to monitor/actuate the turtlebot (from within the container):  
````
cd /AUC-Thesis-DT/OTA_RemoteDrivingConfigurator-main/Designs/
python3 QtGUI.py
````

then configure your dashboard through localhost:8080. 🕹
### 8. Open a new terminal and run a new turtlebot3-noetic image (step 6) then launch the turtlebot3 simulation. 💣
````
source ~/catkin_ws/devel/setup.sh
roslaunch turtlebot3_gazebo turtlebot3_house.launch
````

### 9. On a new terminal window, run another container of the turtlebot3-noetic (step 6) and: 
#### a. Configure Aws for Kinesis Data & Video Streaming [check for help](https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-files.html) then: 

````
bash /AUC-Thesis-DT/RemoteDrivingDashboard-master/cloudconnect.sh
````

### 10. Activate all or any of the following nodes in a new container of the turtlebot3-noetic (step5): 

#### To Launch All nodes at once

````
bash /AUC-Thesis-DT/RemoteDrivingDashboard-master/turtlebot3_nodes.sh 
````
or launch each node on its own
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


### 11. To activate the FMU node: 

#### Replace machine_ip and make sure to source the script with the AMESim env variables (valid only for users who have access to AMESim simcenter).  

````
sudo docker run -it  -v $AME:$AME --env="SPLM_LICENSE_SERVER=28000@orw-sddlic1" --env="SC1D_LICENSING_TYPE=ugs" --env="AME=$AME" --net=ros --env="HOST_IP=<machine_ip>" --env="ROS_MASTER_URI=http://roscore:11311" --env="LD_LIBRARY_PATH=$LD_LIBRARY_PATH" turtlebot3-noetic bash 
````

then: 

````
python3 /AUC-Thesis-DT/ROS-master/cloudconnect/Reactive_FMU.py
````

### 12. Streaming Camera to AWS Kinesis 
````
cd ./Streaming_docker 
docker build -t streaming-node .
docker run -it --env"ROS_MASTER_URI=http://roscore:11311" --net=ros streaming-node bash
````
then
````
nano /opt/ros/kinetic/share/h264_video_encoder/config/sample_configuration.yaml
````
#### and replace `subscription_topic` from `/raspicam_node` to `/camera/rgb/image_raw` 
Configure AWS ([check for help](https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-files.html)). Then: 

````
roslaunch kinesis_video_streamer sample_application.launch
````

and have fun. 🎉



