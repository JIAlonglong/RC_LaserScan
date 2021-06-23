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

# Start ros master
gnome-terminal -t "roscore" -- bash -c "roscore" && echo "Start ros master sucess"

# Wait roscore ready
echo "Waiting delay 15s..."
sleep 15;

# Get laser data
gnome-terminal -t "run_laser" -- bash -c "rosrun urg_node urg_node _ip_address:=192.168.1.11" && echo "Get laser data success"

# Read input to determine whether to calibrate 
read -p "please input whether to calibrate(Y/N):" isCalib
if [ "$isCalib" = "Y" ] || [ "$isCalib" = "y" ]; then
	# Run code
	gnome-terminal -t "run_code" -- bash -c \
	"cd ~/catkin_ws_laser && catkin_make && rosrun pots_coordinates pots_coordinates _calib:=true" &&\
	echo "Start running code "
	
	# Show error 
	gnome-terminal -t "show error" -- bash -c \
	"rosrun pots_coordinates plt_error.py" &&\
	echo "Show error "
else
	# Load param
	gnome-terminal -t "load param" -- bash -c \
	"rosparam load ./src/pots_coordinates/config/calib.yaml" &&\ 	
	echo "load param success"
	
	# Run code
	gnome-terminal -t "run_code" -- bash -c \
	"cd ~/catkin_ws_laser && catkin_make && rosrun pots_coordinates pots_coordinates _calib:=false" &&\
	echo "Start running code "

	# Show pic 
	gnome-terminal -t "show pic" -- bash -c \
	"rosrun pots_coordinates plt_data.py" &&\
	echo "Show pic "

fi




