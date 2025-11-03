gnome-terminal --window -e 'bash -c "sleep 1.0; roslaunch sunray_ugv_control wheeltec_robot.launch  ugv_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=1 location_source:=1 enable_rviz:=false; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_ugv_control ugv_terminal_control.launch ugv_id:=1; exec bash"' \

gnome-terminal --window -e  'bash -c "sleep 15.0; roslaunch sunray_tutorial ugv_test_demo.launch demo_id:=11 ugv_id:=1; exec bash"' \

