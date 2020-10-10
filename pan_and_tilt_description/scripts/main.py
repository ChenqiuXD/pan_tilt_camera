#!/usr/bin/env python3
import rospy
import numpy as np

# from src.pan_and_tilt_description.scripts import ControllerManager
# from src.pan_and_tilt_description.scripts import MarkerManager
# from src.pan_and_tilt_description.scripts import Optimization
import ControllerManager
import MarkerManager
import Optimization
from scipy.spatial import SphericalVoronoi

rospy.init_node("main")
rate = rospy.Rate(1)

# initialization
RADIUS = 100 # The radius of the detection area (a half-ball)
droneArg = [[RADIUS, 0, 1.3, 1.0],
            [RADIUS, 0.8, 1.0, 1.0],
            [RADIUS, 1.7, 1.5, 1.0]]
numofCamera = 4
droneList = MarkerManager.addDrones(droneArg)
markerManager = MarkerManager.MarkerManager(droneList, numofCamera)
ctrlManager = ControllerManager.ControllerManager(numofCamera)
ctrlManager.stateReset()
# Sleep 100 ms. Because ControllerManager need some time to use callback to get the correct states of joints.
rospy.sleep(0.1)

# Draw the half-ball and red area (which represent the probability of drones occuring there)
print("MarkerManager display the half-ball")
# markerManager.display()
markerManager.easy_display()    # A simplified version which only display the red points
print("Start main loop")
# 'test.txt' file is used to store the H values
f = open('test.txt', 'w')

points = np.zeros(shape=(2 * numofCamera, 3))
state = np.zeros(shape=(numofCamera, 2))
count = 0
while not rospy.is_shutdown():
    count = count + 1
    try:
        fov_list = []
        voro_list = []
        # get every camera state (points) and give control command
        print("Getting camera state")
        for i in range(numofCamera):
            # camera state
            state[i] = ctrlManager.getState(i)  # Get camera state
            markerManager.cameras.setAngle(state[i], i) # Update camera state in MarkerManager
            points[i] = RADIUS * MarkerManager.spher2cart(state[i])
            points[numofCamera + i][2] = -points[i][2]
            points[numofCamera + i][1] = points[i][1]
            points[numofCamera + i][0] = points[i][0]

            # camera fov state
            if (count > 1):
                # fov_list are four points representing the four corners of camera image (in sphere coordinate)
                fov_list.append(np.array(
                                    # pi/6 is the "horizontal_fov" written in the body.xacro (robot model)
                                    # The first element is horizontal, the second is vertical
                                    [[np.pi / 6 + state[i][1], np.pi / 6 * (640/480) + state[i][0]],
                                     [np.pi / 6 - state[i][1], np.pi / 6 * (640/480) + state[i][0]],
                                     [np.pi / 6 - state[i][1], np.pi / 6 * (640/480) - state[i][0]],
                                     [np.pi / 6 + state[i][1], np.pi / 6 * (640/480) - state[i][0]]]))  

                # compute ith camera Voronoi region (represented by lines)
                voro_list_i = np.random.rand(0, 3)
                n = len(sv.regions[i])
                for j in sv.regions[i]:
                    voro_list_i = np.concatenate((voro_list_i, [sv.vertices[j]]), axis=0)
                voro_list.append(voro_list_i)

        # Compute the derivation of objective function and control law
        print("Controlling")
        if (count>1):
            speeds = Optimization.controller(state, voro_list, fov_list)
            # speeds *= 10
        # # Saturation of speed
        # if speeds[0] >= 5:
        #     speeds[0] = 5
        # if speeds[1] >= 5:
        #     speeds[1] = 5
            ctrlManager.multimanipulate(speeds)

        # display the camera cone
        markerManager.cameras.pub_range()

        # display the Voronoi regions
        center = np.array([0, 0, 0])
        sv = SphericalVoronoi(points, RADIUS, center)
        MarkerManager.adddisplay(points)
        H_value = Optimization.H(state)
        print("H_function:", H_value)
        
        # Write H value to test.txt file
        f.write(str(H_value))
        f.write(', ')

        rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        f.close()
        rospy.logwarn("ROS Interrupt Exception, trying to shut down node")

if(count >= 400):
    f.close()