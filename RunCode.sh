#!/usr/bin/env bash 

# Author : bonbon
# Script follows here:

# check the serial port and give permission
if [ -e "/dev/ttyUSB0" ];
then    sudo chmod 777 /dev/ttyUSB0 \
       	&& echo "Port /dev/ttyUSB0 ready"
else
        echo "/dev/ttyUSB0 don't exist"
fi

if [ -e "/dev/ttyACM0" ];
then    sudo chmod 777 /dev/ttyACM0 \
		&& echo "Port /dev/ttyACM0 ready"
else
        echo "/dev/ttyACM0 don't exist"
fi


# Read input to determine whether to calibrate 
# read -p "please input whether to calibrate(Y/N):" isCalib
if [ "$1" = "Y" ] || [ "$1" = "y" ]; then
	# calibrate
	gnome-terminal  -- bash -c \
	"catkin_make && roslaunch rc_laserscan calibrate.launch" &&\
	echo "start calibrate"
	
	# Show error 
	gnome-terminal  -- bash -c \
	"rosrun rc_laserscan plt_error.py" &&\
	echo "Show error"

else
	# # Load param
	# gnome-terminal  -- bash -c \
	# "rosparam load ./src/rc_laserscan/config/calib.yaml" &&\
	# echo "load param success"

	#run launch
	gnome-terminal  -- bash -c \
	"catkin_make && roslaunch rc_laserscan start.launch" &&\
	echo "start main"
	
	# Show pic 
	gnome-terminal  -- bash -c \
	"rosrun rc_laserscan plt_data.py" &&\
	echo "Show pic "

	# save 
	gnome-terminal  -- bash -c \
	"cd ./src/rc_laserscan/bag && rosbag record /scan /tf" &&\
	echo "Save to rosbag`"

fi




