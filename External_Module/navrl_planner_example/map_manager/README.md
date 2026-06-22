# 3D Mapping Library For Autonomous Robots
This package is a library implementing various 3D mapping algorithms, such as occupancy voxel map, ESDF Map, [dynamic map](https://ieeexplore.ieee.org/abstract/document/10161194) (our mapping for dynamic environments), for autonomous mobile robots. 

**Authors**: [Zhefan Xu](https://zhefanxu.com/) and Xiaoyang Zhan, Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

If you find this work helpful, kindly show your support by giving us a free ⭐️. Your recognition is truly valued.

This repo can be used as a standalone package and also comes as a module of our [autonomy framework](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy).

## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [onboard_detector](https://github.com/Zhefan-Xu/onboard_detector) which provides the dynamic obstacle detection and tracking for our [dynamic map](https://ieeexplore.ieee.org/abstract/document/10161194). 

```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/map_manager.git

cd ~/catkin_ws
catkin_make
```

If you have catkin_make issue with Eigen package, try the command below:
```
sudo ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen
```

## II. Run DEMO 
a. **Occupancy Map:** In case you do not have/want a hardware platform to play with this repo, we have provided a lightweight [simulator](https://github.com/Zhefan-Xu/uav_simulator.git) for testing. Run the following command to launch the occupancy voxel map:

```
roslaunch uav_simulator start.launch
roslaunch map_manager occupancy_map.launch # check the launch file for ESDF map and dynamic map
roslaunch map_manager rviz.launch
```

The example of running occupancy map is shown as below (note that the robot is controlled by a keyboard):

https://github.com/Zhefan-Xu/map_manager/assets/55560905/40cca490-8613-4d12-ba4a-8b900d3e2cf0

b. **ESDF Map:** The ESDF map is built upon on the occupancy map. The following command runs the ESDF map:

```
roslaunch map_manager ESDF_map.launch
```
The example screenshot of ESDF map is shown below which visualizes the distance field to obstacles:

![ESDF](https://github.com/Zhefan-Xu/map_manager/assets/55560905/e37243c6-eefe-4824-800d-1d8b35aaa74b)

c. **Dynamic Map:** The [dynamic map](https://ieeexplore.ieee.org/abstract/document/10161194) integrates a dynamic object detector to represent both static and moving obstacles in dynamic environments. The following command will launch the dynamic map:


```
roslaunch map_manager dynamic_map.launch
```

The example of the [dynamic map](https://ieeexplore.ieee.org/abstract/document/10161194) is shown below: 

![Screenshot from 2023-12-19 01-07-14](https://github.com/Zhefan-Xu/map_manager/assets/55560905/e9575308-c18f-49b0-9ed3-f5946478c8f5)

The related paper can be found on:

**Zhefan Xu\*, Xiaoyang Zhan\*, Baihan Chen, Yumeng Xiu, Chenhao Yang, and Kenji Shimada, "A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera”, IEEE International Conference on Robotics and Automation (ICRA), 2023.** [\[paper\]](https://ieeexplore.ieee.org/abstract/document/10161194) [\[video\]](https://youtu.be/u5zblVx8KRc?si=3c2AC9mc6pZBUypd).


## III. Parameters
All mapping parameters can be edited and modified in ```map_manager/cfg/***.yaml``` files.

## IV. ROS Topics
- This package subscribes the following topics for occupancy, ESDF, and [dynamic map](https://ieeexplore.ieee.org/abstract/document/10161194):
  - Localization topic: ```robot/odometry``` or ```robot/pose```  (change the topic name in the config file).
  - Depth camera topic: ```camera/depth``` (change the topic name in the config file).
  
- This package publish the following topics:
  - occupancy map visualization: ```occupancy_map/inflated_voxel_map```.
  - esdf map visualization: ```esdf_map/inflated_voxel_map``` and ```esdf_map/esdf```.
  - esdf map visualization: ```dynamic_map/inflated_voxel_map```.

    
## V. Code Example & API
The following example shows the usage our mapping library. Please refer to the source code for more details.
```
#include <map_manager/ESDFMap.h>
#include <map_manager/dynamicMap.h>

int main(){
  ...
  map_manager::ESDFMap m;
  m.initMap(nh);
  
  // collision checking given a point
  Eigen::Vector3d pos (1.0, 1.0, 1.0)
  bool hasCollision = m.isOccupied(pos);
  
  // get distance with gradient (ESDF map)
  Eigen::Vector3d grad;
  double dist = m.getDistanceWithGradTrilinear(pos, grad);


  map_manager::dynamicMap dm;
  dm.init(nh)
  // get dynamic obstacles (dynamic map)
  std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
  dm.getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
  ...
}
```

## VI. Citation and Reference
If you find this work useful, please cite the paper:
```
@inproceedings{xu2023real,
  title={A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera},
  author={Xu, Zhefan and Zhan, Xiaoyang and Chen, Baihan and Xiu, Yumeng and Yang, Chenhao and Shimada, Kenji},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={10645--10651},
  year={2023},
  organization={IEEE}
}
```


