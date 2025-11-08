
gnome-terminal --window -e 'bash -c "roslaunch sunray_fmt_control sunray_mavros_exp.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_fmt_control fmt_external_fusion.launch uav_id:=1 external_source:=3 ; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_fmt_control sunray_fmt_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e  'bash -c "sleep 10.0; roslaunch sunray_tutorial simple_obstacle_avoidance_d435i.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch realsense2_camera rs_camera.launch; exec bash"' \