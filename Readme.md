# A pan-tilt camera  
## Overview  
A pan and tilt camera used in gazebo. The robot is controlled by ROS's controller manager by pid methods. To examine the robot model, please read the 'robot' file and check out pan_and_tilt.xacro. Currently the joint is controlled directly by gazebo by changing velocity.   

## Prequisite
Please copy the 'heatmap' and 'background' models to your gazebo local model database.  
> cp -r `rospack find pan_and_tilt_description`/models/heatmap ~/.gazebo/models    
> cp -r `rospack find pan_and_tilt_description`/models/background ~/.gazebo/models/  

Try 'source devel/setup.bash' if failed, coz we used the `rospack find` command. And note that the local model path may vary.  

## Usage  
Create a workspace, after catkin_make and source the devel/setup.bash, run the following command:  
> roslaunch pan_and_tilt_description main_sim.launch  

To add heatmap models, please run the following command:  
> cd `rospack find pan_and_tilt_description`/scripts  
> python addHeatmaps.py  

(P.S. Currently we are still looking for easier and more convenient way to spawn models :-D ）  
 
Rotate the pan and tilt camera, run:  
> rostopic pub -1 /robot1/pitch_joint_velocity_controlelr/commadn std_msgs/Float64 '{dataL -0.3}'  

or change topic to /pan_and_tilt/yaw_joint_velocity_controller/command to control the base rotation.  
To visualize the camera, run:  
> rosrun image_view image_view image:=/robot1/pan_and_tilt/camera/image_raw  

## Visualization
The pan-and-tilt camera is as following. :) I know it's ugly, please be tolerant  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/Pan_tilt_camera.png)  

The overall scene is:  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/overall_scene.png)  

A camera's view:  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/camera_view.png)  

You can see two robots in rviz:  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/overall_pic.png)  

## Helpful reference:
When launching multiple robots, try following sample:  
> https://answers.gazebosim.org//question/16497/spawning-multiple-robots-each-with-a-controller-in-the-same-namespace-as-the-controller_manager/  

Spawn models by python:  
> https://answers.ros.org/question/246419/gazebo-spawn_model-from-py-source-code/