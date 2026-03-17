gnome-terminal --window -e 'bash -c "roslaunch sunray150 mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray150 sunray_external_fusion.launch external_source:=3; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray150 sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray150 waypoint_mission_node.launch; exec bash"' \
