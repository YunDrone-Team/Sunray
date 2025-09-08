sudo apt-get install ros-noetic-vrpn -y
sudo apt-get install ros-noetic-mavros* -y
sudo cp -rf External_Module/geographic-lib/* /usr/share/GeographicLib
sudo apt-get install ros-noetic-velodyne-gazebo-plugins* -y
sudo apt-get install libignition-math* -y
sudo apt-get install libyaml-cpp-dev -y
sudo apt-get install libsfml-dev -y

sudo apt install tmux
tail -n 1 ~/.tmux.conf | grep -q "set -g mouse on" || echo 'set -g mouse on' | tee -a ~/.tmux.conf
