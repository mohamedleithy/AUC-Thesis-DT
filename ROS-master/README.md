# ROS subsystem (DUT) + Cloud Connect Nodes
ROS subsystem for our case study with turtlebot3 + cloud connect nodes
## Project Demo
The following link demonstrates in a video the whole project process using the configurator:

[Project Demo](https://drive.google.com/file/d/1iktGOEk-YixxjGIhwBbPbenZhpoLa4H1/view)

## Installation

* Install ROS Melodic on your Ubnutu 18 machine
* Clone the repo in catkin_ws/src
* Run this command to build:
```Bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```
## How to Run the ROS subsystem nodes
* Execute this command in order to launch gazebo:
```Bash
export TURTLEBOT3_MODEL=waffle_pi && roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
* Execute this command in order to export the sensors' data to topic /metric:
```Bash
python3 ~/catkin_ws/src/laser_values/src/scan.py
```
* Execute this command in order to export the camera's images to topic /images:
```Bash
python3 ~/catkin_ws/src/sensor_fusion/src/sensor_fusion.py
```
* Execute this command in order to import the actuation direction from topic /actuate:
```Bash
python3 ~/catkin_ws/src/actuate/src/actuate.py
```
## How to run the cloud connect nodes
* Launch Qt configurator: [Configurator Repository](https://github.com/SaraAkmal/OTA_RemoteDrivingConfigurator)
  1. Choose your inputs for the ROS subsystem
  2. Press generate in the configurator to run the dashboard and cloud connect nodes

## Project Features and Details
[Project Thesis](https://github.com/SaraAkmal/RemoteDrivingDashboard/blob/master/Cloud-Connected%20AGV%20Thesis.pdf)

[Project Presentation](https://github.com/SaraAkmal/RemoteDrivingDashboard/blob/master/Cloud-Connected%20AGV%20Presentation.pptx)
