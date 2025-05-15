
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_1uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2 enable_rviz:=true; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e  'bash -c "sleep 5.0; roslaunch sunray_tutorial run_demo.launch demo_id:=2 uav_id:=1; exec bash"' \