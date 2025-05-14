gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_tag_car.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_simulator sim_rviz.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_tutorial qrcode_detection_sim_down.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_tutorial follow_a_car.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; rosrun sunray_tutorial key_move_car.py; exec bash"' \