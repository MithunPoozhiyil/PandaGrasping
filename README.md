
# Franka panda object grasp using Contact graspet

An implementation of object grasping using Franka Panda Robot and Contact graspnet grasp generation algorithm. 


## Prerequisites

- [Contact Graspnet](https://github.com/NVlabs/contact_graspnet) 
- [Segmentation](https://github.com/NVlabs/UnseenObjectClustering)
- [Franka ROS](https://github.com/frankaemika/franka_ros)
- [Realsense ROS](https://github.com/IntelRealSense/realsense-ros)



## Installation
1. Install Contact Graspnet, Segmentation, Franka ROS and Realsense ROS using the links in Prerequisites.
2. Create a python package inside franka_ros using [Panda_Robot_side](https://github.com/MithunPoozhiyil/PandaGrasping/tree/main/Panda_Robot_side)  
3. Copy the contents of [grasp_generation](https://github.com/MithunPoozhiyil/PandaGrasping) into ws_graspnet/src/contact_graspnet_ros/contact_graspnet/contact_graspnet folder.



## Running
Steps
1. Start Camera
Starting camera and making images and depth available in ros topics 
```
roslaunch realsense2_camera rs_aligned_depth.launch tf_prefix:=_camera enable_pointcloud:=true
```

2. Start Segmentation
Using camera ros topics generation of segmentation masks (generated as ros topics) for grasp generation. 

Navigate to UnseenObjectClustering folder
```
cd /home/(your_workspace)../UnseenObjectClustering/
```
Starting Rviz to wait for viewing segmentation output
```

rosrun rviz rviz -d ./ros/segmentation.rviz
```
Start segmentation (parameter '0' represents GPU selection)
```
./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh 0
```
This will start the segementation and outputs can be seen in Rviz. Also associated topics will be publishing the segmented images.

3. Start Grasp generation
To generate grasp positions (will start publisher) using the inputs from camera topics and segmentation topics. The script will be in ws_graspnet/src/contact_graspnet_ros/contact_graspnet/contact_graspnet folder
```
python generate_grasp.py
```

4. Start Franka Panda Robot
5. Start Moveit package
6. manipulating robot for grasping
```
 python panda_move_testmoveit.py
```
