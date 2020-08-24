import rospy
import numpy as np

from src.pan_and_tilt_description.scripts import ControllerManager
from src.pan_and_tilt_description.scripts import MarkerManager
from src.pan_and_tilt_description.scripts import Optimization
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
manager = MarkerManager.MarkerManager(droneList)
points = np.zeros(shape=(2 * numofCamera, 3))
state = np.zeros(shape=(numofCamera, 2))
count = 0
ControllerManager.stateReset(ctrlManager)
center = np.array([0, 0, 0])
# Sleep 100 ms. Because ControllerManager need some time to use callback to get the correct states of joints.
rospy.sleep(0.1)

while not rospy.is_shutdown():
    count = count + 1
    try:
        if (count < 3):
            manager.display()
        """get every camera state(points) and give control command"""
        for i in range(numofCamera):
            """camera state"""
            state[i] = ctrlManager.getState(i)
            points[i] = 20 * MarkerManager.spher2cart(state[i])
            points[numofCamera + i][2] = -points[i][2]
            points[numofCamera + i][1] = points[i][1]
            points[numofCamera + i][0] = points[i][0]
            """camera fov state"""
            fov_list = np.array([[np.pi / 6 + state[i][0], 680 * np.pi / (6 * 480) + state[i][1]],
                                 [np.pi / 6 - state[i][0], 680 * np.pi / (6 * 480) + state[i][1]],
                                 [np.pi / 6 - state[i][0], 680 * np.pi / (6 * 480) - state[i][1]],
                                 [np.pi / 6 + state[i][0], 680 * np.pi / (6 * 480) - state[i][1]]])  # Note

            if (count > 1):
                """compute ith camera Voronoi region"""
                voro_list = np.random.rand(0, 3)
                n = len(sv.regions[i])
                for j in sv.regions[i]:
                    voro_list = np.concatenate((voro_list, [sv.vertices[j]]), axis=0)  # Note
                """Compute the derivation of objective function and control law"""
                speeds = Optimization.controller(state[i], voro_list, fov_list)
                """Avoid speeds is nan"""
                if speeds[0] is np.nan or speeds[1] is np.nan:
                    raise Exception(i, "th speed is nan")
                print(i, "th: ", speeds)
                """saturation of speed"""
                if speeds[0] >= 5:
                    speeds[0] = 5
                if speeds[1] >= 5:
                    speeds[1] = 5
                ctrlManager.manipulate(speeds, i)

        # display
        sv = SphericalVoronoi(points, 20, center)
        MarkerManager.adddisplay(points)
        rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("ROS Interrupt Exception, trying to shut down node")
