gnome-terminal --window -e 'bash -c "roslaunch sunray_fmt_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_fmt_control fmt_external_fusion.launch external_source:=3; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_fmt_control sunray_fmt_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control waypoint_mission_node.launch; exec bash"' \
