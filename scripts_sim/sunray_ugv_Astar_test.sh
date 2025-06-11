gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_simulator sunray_sim_ugv_2d_lidar.launch rviz_enable:=false; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_ugv_control ugv_control_sim_2dlidar.launch; exec bash"' \
