gnome-terminal --window -e 'bash -c "roslaunch web_cam web_front_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_media rtsp_push.launch url:=rtsp://192.168.20.113:8554/live; exec bash"' \

