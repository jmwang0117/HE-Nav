##vins-mono
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch simulation 4x4world.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch simulation model.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch simulation mavros.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_gesture.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch ros_slam cartographer_2Dlidar_location.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \

##--tab -e 'bash -c "sleep 6; roslaunch px4_command 4x4ABCdemo.launch; exec bash"' \
