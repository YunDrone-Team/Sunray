
gnome-terminal --window -e 'bash -c "roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0 enable_rviz:=true; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch rviz:=false; exec bash"'

gnome-terminal --window -e  'bash -c "sleep 15.0; roslaunch sunray_tutorial run_demo.launch demo_id:=4 uav_id:=1; exec bash"' \
