#!/usr/bin/env python3
import rospy
import numpy as np
from Optimization import controller 
from MarkerManager import spher2cart
from scipy.spatial import SphericalVoronoi
import ControllerManager
import MarkerManager
import Optimization

def getCmd(states):
    """
    Generate control command based on state and droneArg  
    This is used in the real cameras  
    Parameters  
    ----------
    states      :   states of camera [ [pitch1, yaw1], ... ]  
    Returns
    -------
    the velocity commands of cameras [ [pitch_vel_1, yaw_vel_1], ... ]  
    """
    numOfCam = states.shape[0]
    RADIUS = 100

    # Update the voronoi region
    points = np.zeros(shape=(2*numOfCam, 3))   
    for i in range(numOfCam):
        # Updating points
        points[i] = RADIUS * spher2cart(states[i])
        points[numOfCam + i][2] = -points[i][2]
        points[numOfCam + i][1] = points[i][1]
        points[numOfCam + i][0] = points[i][0]
    center = np.array([0, 0, 0])
    sv = SphericalVoronoi(points, RADIUS, center)

    # Get fov_list and voro_list
    fov_list = []
    voro_list = []     
    for i in range(numOfCam):
        fov_list.append(np.array(
                            # The first element is horizontal, the second is vertical
                            [[np.pi / 6 + states[i][1], np.pi / 6 * (640/480) + states[i][0]],
                             [np.pi / 6 - states[i][1], np.pi / 6 * (640/480) + states[i][0]],
                             [np.pi / 6 - states[i][1], np.pi / 6 * (640/480) - states[i][0]],
                             [np.pi / 6 + states[i][1], np.pi / 6 * (640/480) - states[i][0]]]))  

        voro_list_i = np.random.rand(0, 3)
        for j in sv.regions[i]:
            voro_list_i = np.concatenate((voro_list_i, [sv.vertices[j]]), axis=0)
        voro_list.append(voro_list_i)

    # Get the controller command
    speeds = controller(states, voro_list, fov_list)

    return speeds, points

if __name__ == "__main__":
    rospy.init_node("main")
    rate = rospy.Rate(1)
    RADIUS = 100
    MAX_ITER = 400
    # Note that if you wanna change the droneArg, please check the following places:
    # Optimization.py line328 droneArg (in phi function)
    droneArg = [[RADIUS, 0.23, 0.34, 1.0], 
                [RADIUS, 1.5, 0.34, 1.0],
                [RADIUS, 0, 0, 1.0]]
    numofCamera = 4
    droneList = MarkerManager.addDrones(droneArg)
    markerManager = MarkerManager.MarkerManager(droneList, numofCamera)
    ctrlManager = ControllerManager.ControllerManager(numofCamera)
    ctrlManager.stateReset()
    # Sleep 100 ms. Because ControllerManager need some time to use callback to get the correct states of joints.
    rospy.sleep(0.1)
    markerManager.easy_display()    # A simplified version which only display the red points

    count = 0
    state = np.zeros(shape=(numofCamera, 2))
    while not rospy.is_shutdown() and count <= MAX_ITER:
        for i in range(numofCamera):
            state[i] = ctrlManager.getState(i)
            markerManager.cameras.setAngle(state[i], i) # Update camera state in MarkerManager
        speeds, points = getCmd(state)
        ctrlManager.multimanipulate(speeds)
        MarkerManager.adddisplay(points)

        H_value = Optimization.H(state)
        print("H_function:", H_value)
        rate.sleep()