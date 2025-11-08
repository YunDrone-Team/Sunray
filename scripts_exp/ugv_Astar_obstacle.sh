gnome-terminal --window -e 'bash -c "sleep 1.0; roslaunch sunray_ugv_control wheeltec_robot.launch  ugv_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=1 location_source:=1 enable_rviz:=false; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_ugv_control ugv_terminal_control.launch ugv_id:=1; exec bash"' \

gnome-terminal --window -e  'bash -c "sleep 3.0; roslaunch oradar_lidar ms200.launch; exec bash"' \
