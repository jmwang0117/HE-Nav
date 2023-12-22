##vins-mono
gnome-terminal --window -e 'bash -c "roslaunch mavros px4_ego.launch gcs_url:=udp://@192.168.10.102; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_d400_and_t265.launch filters:=pointcloud pointcloud_texture_stream:=RS2_STREAM_ANY; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/Vi2ENav/devel/setup.bash;roslaunch pubcmd cam2base.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/Vi2ENav/devel/setup.bash;rosrun pubcmd pub_campose.py; exec bash"' \
--tab -e 'bash -c "sleep 12;source ~/Vi2ENav/devel/setup.bash; roslaunch ego_planner px4_demo.launch; exec bash"' \
--tab -e 'bash -c "sleep 15;source ~/Vi2ENav/devel/setup.bash; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 15;source ~/Vi2ENav/devel/setup.bash; roslaunch px4_command px4_replan_sender.launch; exec bash"' \
--tab -e 'bash -c "sleep 15;source ~/Vi2ENav/devel/setup.bash; roslaunch px4_command auto_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 15;source ~/Vi2ENav/devel/setup.bash; rosrun pubcmd pubcmd.py; exec bash"' \
--tab -e 'bash -c "sleep 5;source ~/Vi2ENav/devel/setup.bash; roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch; exec bash"' \
--tab -e 'bash -c "sleep 10;source ~/Vi2ENav/devel/setup.bash; rosrun pubcmd publish_wheeltec_twist.py; exec bash"' \
--tab -e 'bash -c "sleep 15;source ~/Vi2ENav/devel/setup.bash; rostopic echo planning/pos_cmd; exec bash"'




#--tab -e 'bash -c "sleep 12;source ~/Vi2ENav/devel/setup.bash; roslaunch ego_planner rviz.launch; exec bash"' \
