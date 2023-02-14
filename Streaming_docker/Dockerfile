# This is an example Docker File
#  Command to build it
# docker built -t <image name > .

#Start from ros:kinetic-desktop-full image

FROM osrf/ros:kinetic-desktop-full

#install necessary packages 
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip && apt-get install -y curl  && \ 
apt-get install -y wget && apt-get install -y net-tools \
&& apt-get install -y nano && apt-get install -y vim


RUN apt-get install -y ros-$ROS_DISTRO-h264-video-encoder
RUN sudo apt-get install -y ros-$ROS_DISTRO-kinesis-video-streamer 

RUN apt-get -y install unzip

RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
RUN unzip awscliv2.zip
RUN ./aws/install

RUN echo "ALL Done"



