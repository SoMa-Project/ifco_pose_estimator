#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

ROS_DISTRO=$(ls /opt/ros/) 

printf "Checking dependencis\n"
printf "distro: ${ROS_DISTRO}\n"

install_library(){
	LIBRARY=$(dpkg -S $1 )
	if [[ -z $LIBRARY  ]]; 
	then
		printf "${GREEN}installing $1\n${NC}"
		apt-get -qq --yes --force-yes install $1
	else
		printf "${GREEN}$1 ... ok\n${NC}"
	fi
}

install_library ros-${ROS_DISTRO}-ros-core
install_library ros-${ROS_DISTRO}-pcl-ros
install_library ros-${ROS_DISTRO}-moveit