gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_simulator sunray_sim_1ugv.launch rviz_enable:=false; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_ugv_control ugv_control_sim.launch; exec bash"' \
