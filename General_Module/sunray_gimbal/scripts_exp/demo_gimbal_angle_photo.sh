gnome-terminal --window -e  'bash -c "sleep 1.0; roslaunch sunray_gimbal gimbal_control.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 8.0; rosrun sunray_gimbal gimbal_angle_photo; exec bash"' \



