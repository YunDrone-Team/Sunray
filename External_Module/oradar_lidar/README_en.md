# ORADAR ROS package

The ORADAR ROS package is used to connect the Oradar MS200 LiDAR. This ROS package supports ROS and ROS2. ROS supports Indigo, Kinetic, Melodic and other ROS versions, and ROS2 supports Ubuntu 20.04 ROS2 foxy version and above.

## Guides： 

1. Install the ROS environment on the system, refer to the following link for details:

   ROS installation link:  http://wiki.ros.org/kinetic/Installation/Ubuntu 

   RROS2 installation link: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

   **It is recommended not to install ROS and ROS2 on one computer at the same time to avoid possible version conflicts and the trouble of manually installing other libraries**

2. Copy the oradar_ros source code to the src directory under the ros working directory, and modify the corresponding file

   ```shell
   mkdir -p ~/lidar_ros_ws/src
   cp -ar oradar_ros ~/lidar_ros_ws/src/
   ```

   (1) When using ROS, you need to open the *CMakeLists.txt* file in the root of the oradar_ros source code and change the variable **COMPILE_METHOD** at the top of the file to **CATKIN**

   ```cmake
   #=======================================
   # Compile setup (ORIGINAL,CATKIN,COLCON)
   #=======================================
   set(COMPILE_METHOD CATKIN)
   ```

   then, copy the *package_ros1.xml* file and name it *package.xml*.

   (2) When using ROS2, you need to open the *CMakeLists. txt* file in the root directory of the oradar_ros source code, and Change the variable **COMPILE_METHOD** at the top of the file to **COLCON**

   ```cmake
   #=======================================
   # Compile setup (ORIGINAL,CATKIN,COLCON)
   #=======================================
   set(COMPILE_METHOD COLCON)
   ```

   then, copy the *package_ros2.xml* file and name it *package.xml*.


3. Compile the project and set environment variables

   ROS:

   ```shell
   cd ~/lidar_ros_ws
   catkin_make
   source devel/setup.bash
   ```

   ROS2:

   ```
   cd ~/lidar_ros_ws
   colcon build
   source install/setup.bash
   ```

4. Configure the serial port of the upper computer

   Configure serial "port_name" and "baudrate": default configuration port_name  is "/dev/ttyUSB0", and the baud rate is "230400"

5. Configure LiDAR parameters

   Open "oradar_ros/launch/ms200_scan.launch" or "oradar_ros/launch/ms200_scan.launch.py" for parameter configuration

   The parameters are described as follows:

   | Parameter | data type | description                                              |
   | ----------- | -------- | ------------------------------------------------------------ |
   | frame_id    | string   | Lidar coordinate system name. Default is "laser_frame" |
   | scan_topic  | string   | LaserScan topic name. Default is "scan" |
   | port_name   | string   | The name of the Lidar serial port. Default value is "/dev/ttyUSB0" |
   | baudrate    | int      | Lidar serial baud rate. Default value is "230400" |
   | angle_min   | double   | Minimum angle, unit degree, value range [0, 360]. Default value is "0" |
   | angle_max   | double   | Maximum angle, unit degree, value range [0, 360]. Default value is "360" |
   | range_min   | double   | Minimum distance, in meters, Default value is "0.05" |
   | range_max   | double   | Maximum distance, in meters, Default value is "20.0" |
   | clockwise    | bool     | Configure the direction of the point cloud. True is clockwise, and false is counterclockwise. Default value is "false" |
   | motor_speed | int      | Lidar speed, unit: Hz, value range: 5~15Hz. Default value is "10Hz" |

   

6. Start the Oradar ros node

   **When the environment is ROS:**

   1. Publish  "LaserScan"

   ```shell
   roslaunch oradar_lidar ms200_scan.launch
   OR
   roslaunch oradar_lidar ms200_scan_view.launch(displayed by rviz)
   ```
   2. Publish "PointCloud"
   ```shell
   roslaunch oradar_lidar ms200_pointcloud.launch
   OR
   roslaunch oradar_lidar ms200_pointcloud_view.launch(displayed by rviz)
   ```   

   **When the environment is ROS2:**

   1. Publish  "LaserScan"
   ```
   ros2 launch oradar_lidar ms200_scan.launch.py
   OR
   ros2 launch oradar_lidar ms200_scan_view.launch.py(displayed by rviz2)
   ```
   2. Publish "PointCloud"
   ```
   ros2 launch oradar_lidar ms200_pointcloud.launch.py
   OR
   ros2 launch oradar_lidar ms200_pointcloud_view.launch.py(displayed by rviz2)
   ```

   
