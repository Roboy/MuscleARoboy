# MuscleARoboy
Augmented reality based muscle visualization of Roboy

# Overview
First version of the MuscleARoboy project. The project aims to overlay muscles over Roboy. For this, 3d pose estimation and a see-through head mounted display are required.
In the current version you can find ROS nodes for the following functions:
* allow using an Android smartphone with Google Cardboard as see-through HMD with ROS Kinetic
* building of a 3d model and generating virtual views from different perspectives to build view database ("Offline part")
* Work in progress - comparison of the view from the HMD with the view database to find most likely pose ("Online part")

# Dependencies
* Linux - tested on Arch (03/2017), should work with Ubuntu 16.04 and other linux distributions
* ROS Kinetic with at least the following modules/stacks: ros_core, ros_comm, image_common, image_pipeline, vision_opencv (opencv3), geometry, openni2_camera, openni2_launch, viz (rviz), perception_pcl, rtabmap

# Usage
All nodes have corresponding launch files that show sample usage. These are recommended to start the project, but the nodes can also be run standalone. 
Check the parameters in the nodes and launch files that can be set via commandline.

Sample commands:

```
roslaunch musclearoboy view_generation.launch image_output_path:="/home/USERNAME/images/"

rosrun musclearoboy cloud_publisher _pcd_path:="/home/USERNAME/roboy.pcd"
```

## Launch Files - online part:
* smartphone_hmd.launch: starts the streaming between smartphone and ros. publishes incoming image on ros topic (parameter), receives modified image from ros topic (parameter), simulates stereo and prepares for streaming back via mjpg-streamer

## Launch files - offline preparation:
* openni2_recording.launch: launches the openni2 driver and rosbag record with the appropriate topics
* rtabmap_bag.launch: launches a paused bag file, the dry openni2 driver and RTABMap. After everything loaded, press space while focusing the terminal to start playback of the bag file (needs to be captured with openni2 driver)
* cloud_publisher.launch: publishes a specified pointcloud for usage in ros. can be viewed e.g. in RVIZ
* view_generation.launch: includes cloudpublisher.launch and runs the virtualprojection module to generate virtual views of the pointcloud as images in the specified folder