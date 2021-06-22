# check the serial port and give permission
if [ -e "/dev/ttyUSB0" ];
then    sudo chmod 777 /dev/ttyUSB0 \
       	&& echo "/dev/ttyUSB0 exist"
else
        echo "/dev/ttyUSB0 not exist"
fi

if [ -e "/dev/ttyACM0" ];
then    sudo chmod 777 /dev/ttyACM0 && echo "Port /dev/ttyACM0 ready"
else
        echo "/dev/ttyACM0 not exist"
fi

# start ros master
gnome-terminal -t "roscore" -- bash -c "roscore" && echo "Start ros master sucess" && echo "Waiting delay 15s...";
sleep 15;

#get laser data
gnome-terminal -t "run_laser" -- bash -c "rosrun urg_node urg_node _ip_address:=192.168.1.11" && echo "Get laser data success"

#load param
gnome-terminal -t "load param" -- bash -c "rosparam load ./src/pots_coordinates/config/calib.yaml" && echo "load param success "

# run code
gnome-terminal -t "run_code" -- bash -c "cd ~/catkin_ws_laser && catkin_make && rosrun pots_coordinates pots_coordinates" && echo "Start running code "

# show pic 
# gnome-terminal -t "show pic" -- bash -c "rosrun pots_coordinates plt_data.py" && echo "Show pic "

# show error 
gnome-terminal -t "show error" -- bash -c "rosrun pots_coordinates plt_error.py" && echo "Show error "
