gnome-terminal --window -e  'bash -c "sleep 2.0; roslaunch sunray_gimbal gimbal_control.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_gimbal gimbal_qrtracker.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_tutorial qrcode_detection.launch; exec bash"' \


