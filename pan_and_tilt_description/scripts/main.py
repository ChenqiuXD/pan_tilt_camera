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

import time

rospy.init_node("main")
rate = rospy.Rate(1)

# initialization
droneArg = [[20, 0.23, 0.34, 1],
            [20.3, 1.5, 0.34, 1],
            [20.9, 0, 0, 1]]
numofCamera = 4
droneList = MarkerManager.addDrones(droneArg)
ctrlManager = ControllerManager.ControllerManager(numofCamera)
manager = MarkerManager.MarkerManager(droneList)
points = np.zeros(shape=(2 * numofCamera, 3))
state = np.zeros(shape=(numofCamera, 2))
count = 0
ControllerManager.stateReset(ctrlManager)
center = np.array([0, 0, 0])
# Sleep 100 ms. Because ControllerManager need some time
# to use callback to get the correct states of joints.
rospy.sleep(0.1)

start = time.time()

while not rospy.is_shutdown():
    count = count + 1
    try:
        fov_list = []
        voro_list = []
        if (count < 3):
            manager.display()
        print ("Display time: ", time.time()-start)
        start = time.time()
        # get every camera state(points) and give control command
        for i in range(numofCamera):
            # camera state
            state[i] = ctrlManager.getState(i)
            points[i] = 20 * MarkerManager.spher2cart(state[i])
            points[numofCamera + i][2] = -points[i][2]
            points[numofCamera + i][1] = points[i][1]
            points[numofCamera + i][0] = points[i][0]

            # camera fov state
            if (count > 1):
                # Please check Optimizatioin.py to see the definition of pi/6 and 680*pi/(6*480)
                fov_list.append(np.array(
                                    [[np.pi / 6 + state[i][0], 680 * np.pi / (6 * 480) + state[i][1]],
                                     [np.pi / 6 - state[i][0], 680 * np.pi / (6 * 480) + state[i][1]],
                                     [np.pi / 6 - state[i][0], 680 * np.pi / (6 * 480) - state[i][1]],
                                     [np.pi / 6 + state[i][0], 680 * np.pi / (6 * 480) - state[i][1]]]))  # Note
                # compute ith camera Voronoi region
                voro_list_i = np.random.rand(0, 3)
                n = len(sv.regions[i])
                for j in sv.regions[i]:
                    voro_list_i = np.concatenate((voro_list_i, [sv.vertices[j]]), axis=0)  # Note
                voro_list.append(voro_list_i)

        print ("Got access to states: ", time.time()-start)
        start = time.time()

        # Compute the derivation of objective function and control law
        if (count>1):
            print ("state is: \n", state )
            print ("voro_list is: \n", voro_list)
            print ("fov_list is: \n", fov_list)
            speeds = Optimization.controller(state, voro_list, fov_list)
        # # saturation of speed
        # if speeds[0] >= 5:
        #     speeds[0] = 5dawsh 
        # if speeds[1] >= 5:
        #     speeds[1] = 5
            ctrlManager.multimanipulate(speeds)

        print ("Calculate the command: ", time.time()-start)
        start = time.time()

        # display
        sv = SphericalVoronoi(points, 20, center)
        MarkerManager.adddisplay(points)
        print("H_function:", Optimization.H(state))
        rate.sleep()

        print("Display at last: ", time.time()-start)
        start = time.time()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("ROS Interrupt Exception, trying to shut down node")
