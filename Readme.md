# A pan-tilt camera  
## Overview  
A pan and tilt camera used in gazebo. The robot is controlled by ROS's controller manager by pid methods. To examine the robot model, please read the 'robot' file and check out pan_and_tilt.xacro. Currently the joint is controlled directly by gazebo by changing velocity.   

## Usage  
Create a workspace, after catkin_make and source the devel/setup.bash, run the following command:  
> roslaunch pan_and_tilt_description main_sim.launch  

Rotate the pan and tilt camera, run:  
> rostopic pub -1 /pan_and_tilt/pitch_joint_velocity_controller/command std_msgs/Float64 '{data: -0.3}'  
> Or with multiple robots  
> rostopic pub -1 /robot1/pitch_joint_velocity_controlelr/commadn std_msgs/Float64 '{dataL -0.3}'  

or change topic to /pan_and_tilt/yaw_joint_velocity_controller/command to control the base rotation.  
To visualize the camera, run:  
> rosrun image_view image_view image:=/pan_and_tilt/camera/image_raw  
> Or with multiple robots  
> rosrun image_view image_view image:=/robot1/pan_and_tilt/camera/image_raw  

## Visualization
The pan-and-tilt camera is like :) I know it's ugly, be tolerant  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/Pan_tilt_camera.png)  

The overall scene is:  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/overall_scene.png)  

A camera's view:  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/camera_view.png)  

## Helpful reference:
When launching multiple robots, try following sample:  
> https://answers.gazebosim.org//question/16497/spawning-multiple-robots-each-with-a-controller-in-the-same-namespace-as-the-controller_manager/  
