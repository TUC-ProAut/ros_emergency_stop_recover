**ROS Emergency Stop and Recover**
-
This repository is ROS implementation of Emergency Stop module and Recovery module for autonomous robot. 

**Problem Statement** 

In indoor environments crowded with people, an important factor is to consider safety of robot. Sometimes there are people moving too close to an autonomos robot, and as as a standard solution robot triggers emergency stop. But there are no strategies available to as to how robot recovers or move itself after emergency stop. We propose a simple solution for robot safety, which will not only implement emergency stop of the robot but also recover the robot from emergency stop mode. Also, keeping in check that recovery process will not affect the autonomously robot pursuing its current goal.


**Solution**
1. If any object is in robot's close vicinty ( designated Safety Region), robot will first implement an emeregency stop. 
2. Robot will push itself in oppsite direction of the object entered in safety region, until there is no object present in the safety region.
3. When there are no objects present in safety region, robot will resume its normal operation. 

**Demo:**

![Demo Gif](./demo.gif)

**Input:**  point cloud topic (default = /input_cloud)

**Output:** recover velocity published on topic (default = /cmd_vel)

**Configurations:** 

1. safety region: Dimensions of safety region, read more [here](./ros_emergency_stop_recover/launch/emergency_stop_recover.launch)

2. src/config/twist_mux.yaml: append the all acting velocities on robot, with highest priority geven to recover/cmd_vel and all the robot is subscribing to '/cmd_vel' topic.

3. src/config/params.cfg: 

- pointcloudTopic: Topic Name of input pointcloud
- baseLinkFrame: Frame ID of the base link
- velocityTopic: Topic for publishing recover velocity
- inlierThresh: Threshold for inlier counter ( minimum points in safety region to operate emeregency and stop recover)
- recover_step_vel: Step Increment for recover velocity
- recover_factor_vel: Multiplying factor for recover velocity

**Usage:**  $ roslaunch ros_emergency_stop_recover emergency_stop_recover

**Key Points**

1. Since we are muxing all the command velocities acting on the robot, there will be no loss of preset navigation goal
2. Safety region around should be smaller than local cost map, else the robot will move oscillate between recover velocity and navigation velocity
