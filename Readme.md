# A pan-tilt camera networks 
## Overview  
A pan and tilt camera networks used in gazebo. The robot is controlled by ROS's controller manager by pid methods. To examine the robot model, please read the 'robot' file and check out pan_and_tilt.xacro. Currently the joint is controlled directly by gazebo by changing velocity.   

## Prequisite
Please copy the 'heatmap' and 'background' models to your gazebo local model database.  
> cp -r `rospack find pan_and_tilt_description`/models/heatmap ~/.gazebo/models    
> cp -r `rospack find pan_and_tilt_description`/models/background ~/.gazebo/models/  
> cp -r `rospack find pan_and_tilt_description`/models/skyscraper ~/.gazebo/models/  

Try 'source devel/setup.bash' if prompted "Error: package 'pan_and_tilt_description' not found", coz we used the `rospack find` command. And note that the local model path may vary.  

## Usage  
Create a workspace, after catkin_make and source the devel/setup.bash, run the following command:  
> roslaunch pan_and_tilt_description main_sim.launch  
or  
> roslaunch pan_and_tilt_description main_sim.launch gui:=true  
If you wish to see the gazebo window.  

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

You can see four robots in rviz:  
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/overall_pic.png)  

## Simulation setup
In real-world applications, it can be used to detect air vehicle invasion. In particular, we will introduce the configuration of the PT-camera model programmed by ourselves, our experiment setting, and the numerical implementation of our optimization algorithm. 
 
We programmatically create a PT-camera Gazebo model which can move in the horizontal direction or vertical direction. The camera has a 60-degree FOV  horizontally and every image frame has 680×480 pixels. There are four cameras in total, placed 1.5m apart from each other located on the roof of a building.
 
They are required to cover a half of sphere (airspace) with the radius being 100m. We set a prior event distribution function over this sphere as
![Alt text](https://latex.codecogs.com/svg.latex?\phi(q)=\sum_{i=1}^m\exp({-D_i\|q-o_i\|^2}))

where m is the number of regions that are apt to be invaded by air vehicles with high possibilities, o_i represents the spot centered at each region i, and D_i is a pre-defined distribution decay coefficient. Particularly, in our experiment we set m=3, and for each o_i the parameters are  explicitly given as follows:
![Alt text](https://latex.codecogs.com/svg.latex?space;o_1=[-42.5,~72.6,~54.0]^\top,D_1=0.05,o_2=[26.7,~96.4,~0]^\top,D_2=0.05,o_3=[-57.3,~-81.6,~7.1]^\top,D_3=0.05.)

We use Monte-Carlo integration to calculate the integral calculations. The details to calculate the gradient on the sphere are reminded in our paper. 

The results of our simulations are shown:
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/beginning.png) 
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/timing.png) 
![Alt text](https://github.com/ChenqiuXD/pan_tilt_camera/blob/master/pics/ending.png) 
This depict three snapshots of the camera network coverage cells. It indicates that eventually the cameras will point to the neighborhood of the red regions, of which the event distribution is higher. When you update the event distribution, camera networks will update their attitudes as well. Note that if you want to change the event distribution on this sphere, you must modify "" in  optimization.py and main.py.


## Helpful reference:
When launching multiple robots, try following sample:  
> https://answers.gazebosim.org//question/16497/spawning-multiple-robots-each-with-a-controller-in-the-same-namespace-as-the-controller_manager/  

Spawn models by python:  
> https://answers.ros.org/question/246419/gazebo-spawn_model-from-py-source-code/
