# This is an example Docker File
#  Command to build it
# docker built -t <image name > .

#Start from ros:noetic-desktop-full image

FROM osrf/ros:noetic-desktop-full

#install necessary packages 
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip && apt-get install -y curl  && \ 
apt-get install -y wget && apt-get install -y net-tools \
&& apt-get install -y nano && apt-get install -y vim
RUN pip3 install python-socketio && pip3 install requests && pip3 install boto3


# software packages from ROS 

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update


#install packages for TURTLEBOT3_MODEL=wafflebot3 

RUN apt-get -y install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

RUN apt-get -y install ros-noetic-dynamixel-sdk
RUN apt-get -y install ros-noetic-turtlebot3-msgs
RUN apt-get -y install ros-noetic-turtlebot3
RUN apt-get -y  install docker-compose


# simulation 
RUN mkdir -p ~/catkin_ws/src/turtlebot3_simulations
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git ~/catkin_ws/src/turtlebot3_simulations
RUN git clone https://github.com/mohamedleithy/AUC-Thesis-DT.git
RUN wget https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_description -P ~/catkin_ws/src/turtlebot3_simulations
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws/; catkin_make'
RUN cp -r /AUC-Thesis-DT/ROS-master/actuate ~/catkin_ws/src
RUN cp -r /AUC-Thesis-DT/ROS-master/laser_values ~/catkin_ws/src
RUN cp -r /AUC-Thesis-DT/ROS-master/sensor_fusion ~/catkin_ws/src
RUN pip3 install -r /AUC-Thesis-DT/RemoteDrivingDashboard-master/requirements.txt
RUN . ~/catkin_ws/devel/setup.sh 



RUN apt-get -y install unzip

RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
RUN unzip awscliv2.zip
RUN ./aws/install







RUN echo "ALL Done"



