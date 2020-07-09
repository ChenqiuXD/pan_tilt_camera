# A pan-tilt camera  
## Overview  
A pan and tilt camera used in gazebo. The robot is controlled by ROS's controller manager by pid methods. To check the robot model, please read the 'robot' file and check pan_and_tilt.xacro. To modify the pid parameters, please change the pan_and_tilt.yaml in the config folder.  

## Usage  
Create a workspace, catkin_make and source the devel/setup.bash, run the following command:  
> roslaunch pan_and_tilt_description main_sim.launch  
Rotate the pan and tilt camera, run:  
> rostopic pub -1 /pan_and_tilt/pitch_joint_velocity_controller/command std_msgs/Float64 '{data: -0.3}'  
or change topic to /pan_and_tilt/yaw_joint_velocity_controller/command to control the base rotation.  
To visualize the camera, run:  
> rosrun image_view image_view image:=/pan_and_tilt/camera/image_raw  

## Visualization
The pan-and-tilt camera is like:(know it's ugly, be tolerant)    
[Pan_tilt_camera.png]  
The overall scene is:  
[overall_scene.png]  