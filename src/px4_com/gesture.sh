##vins-mono
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch simulation world.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch simulation model.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch simulation mavros.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_gesture.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch ros_slam cartographer_2Dlidar_location.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/amov/detect_ws/devel/setup.bash; roslaunch darknet_ros darknet_ros.launch"' \

##--tab -e 'bash -c "sleep 6; roslaunch px4_command crossing_door_normal.launch; exec bash"' \
##--tab -e 'bash -c "sleep 6; roslaunch px4_command crossing_door_darknet.launch; exec bash"' \

