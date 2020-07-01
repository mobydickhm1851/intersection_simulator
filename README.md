# Intersection Simulator in Gazebo
This package help one build a simulated intersection, where two Toyota Prius, driven by two human drivers, are traffic participants that interact with each other. 
With all essential states being recorded, Probability of Yielding (POY) are then calculated and analyzed.

## Environment Setup
- Ubuntu: 16.04 LTS
- ROS: Kinetics
- Gazebo: Gazebo 7
- Python: python 2.7

## File System
- prius_control
This folder contain essential setting and launch file for controllers in Gazebo.

- prius_gazebo
This is used as the main folder for this package, with launch files that brings up simulated environment and vehicles that allows virtual experiments.
Data collection are incorporated in the scripts files, which are recommanded to be used with rosbag file. However, real time collection and plotting is also an option.

- prius_description
All models are here including xacro/macro files, urdf, and gazebo files.

## Launch
To spawn the intersection model,
```
roslaunch prius_gazebo POS_intersection.launch
```
One can control prius using both joy sticks or keyboard. In the default setting, prius is controled using throttle and brake and three transition are D/N/R, for forward, nutral and reverse.


