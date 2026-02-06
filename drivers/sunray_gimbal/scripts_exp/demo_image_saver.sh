gnome-terminal --window -e  'bash -c "sleep 1.0; roslaunch sunray_gimbal gimbal_control.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5.0; rosrun sunray_gimbal image_saver; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_gimbal gimbal_qrtracker.launch; exec bash"' \



