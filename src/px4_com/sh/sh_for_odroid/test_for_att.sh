##
mate-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch vrpn_client_ros sample.launch server:=192.168.1.128; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:921600"; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command px4_pos_att_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_command move; exec bash"' \

