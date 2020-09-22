#!/usr/bin/env python3
import rospy
import numpy as np

from src.pan_and_tilt_description.scripts import ControllerManager
from src.pan_and_tilt_description.scripts import MarkerManager
from src.pan_and_tilt_description.scripts import Optimization
# import ControllerManager
# import MarkerManager
# import Optimization
from scipy.spatial import SphericalVoronoi

rospy.init_node("main")
rate = rospy.Rate(1)

# initialization
droneArg = [[20, 0.23, 0.34, 1],
            [20.3, 1.5, 0.34, 1],
            [20.9, 0, 0, 1]]
numofCamera = 4
droneList = MarkerManager.addDrones(droneArg)
ctrlManager = ControllerManager.ControllerManager(numofCamera)
dispManager = MarkerManager.MarkerManager(droneList)
points = np.zeros(shape=(2 * numofCamera, 3))
state = np.zeros(shape=(numofCamera, 2))
count = 0
ControllerManager.stateReset(ctrlManager)
# Sleep 100 ms. Because ControllerManager need some time to use callback to get the correct states of joints.
rospy.sleep(0.1)

# Draw the half-ball and red area (which represent the probability of drones occuring there)
dispManager.display()
print("Start main loop")
while not rospy.is_shutdown():
    count = count + 1
    try:
        fov_list = []
        voro_list = []
        # get every camera state(points) and give control command
        print("Getting camera state")
        for i in range(numofCamera):
            # camera state
            state[i] = ctrlManager.getState(i)
            points[i] = 20 * MarkerManager.spher2cart(state[i])
            points[numofCamera + i][2] = -points[i][2]
            points[numofCamera + i][1] = points[i][1]
            points[numofCamera + i][0] = points[i][0]

            # camera fov state
            if (count > 1):
                # fov_list are four points representing the four corners of camera image (in sphere coordinate)
                fov_list.append(np.array(
                                    # pi/6 is the "horizontal_fov" written in the body.xacro (robot model)
                                    # THe first element is horizontal, the second is vertical
                                    [[np.pi / 6 + state[i][0], np.pi / 6 * (640/480) + state[i][1]],
                                     [np.pi / 6 - state[i][0], np.pi / 6 * (640/480) + state[i][1]],
                                     [np.pi / 6 - state[i][0], np.pi / 6 * (640/480) - state[i][1]],
                                     [np.pi / 6 + state[i][0], np.pi / 6 * (640/480) - state[i][1]]]))  # Note
                # compute ith camera Voronoi region (represented by lines)
                voro_list_i = np.random.rand(0, 3)
                n = len(sv.regions[i])
                for j in sv.regions[i]:
                    voro_list_i = np.concatenate((voro_list_i, [sv.vertices[j]]), axis=0)  # Note
                voro_list.append(voro_list_i)

        # Compute the derivation of objective function and control law
        print("Controlling")
        if (count>1):
            speeds = Optimization.controller(state, voro_list, fov_list)
        # # Saturation of speed
        # if speeds[0] >= 5:
        #     speeds[0] = 5
        # if speeds[1] >= 5:
        #     speeds[1] = 5
            ctrlManager.multimanipulate(speeds)

        # display the Voronoi regions
        center = np.array([0, 0, 0])
        sv = SphericalVoronoi(points, 20, center)
        MarkerManager.adddisplay(points)
        print("H_function:", Optimization.H(state))
        rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("ROS Interrupt Exception, trying to shut down node")