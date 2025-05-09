gnome-terminal --window -e 'bash -c "roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch web_cam web_cam.launch ; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control external_fusion.launch external_source:=3; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_tutorial qrcode_detection_down.launch ; exec bash"' \
--tab -e 'bash -c "sleep 5.0; rqt_image_view; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_tutorial follow_a_car.launch; exec bash"' \
# --tab -e 'bash -c "sleep 8.0; rosrun sunray_tutorial key_move_car.py; exec bash"' \