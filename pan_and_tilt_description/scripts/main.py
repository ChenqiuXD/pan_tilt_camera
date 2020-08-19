import rospy
import numpy as np

from scripts import ControllerManager
from scripts import MarkerManager
from scripts import Optimization

rospy.init_node("main")
rate=rospy.Rate(1)

droneArg = [[20, 0.23, 0.34, 1],
            [20.3, 1.5, 0.34, 1],
            [20.9, 0, 0, 1]]
numofCamera = 4
droneList = MarkerManager.addDrones(droneArg)

ctrlManager = ControllerManager.ControllerManager(numofCamera)
manager = MarkerManager.MarkerManager(droneList)
points = np.zeros(shape=(numofCamera, 3))
state = np.zeros(shape=(numofCamera, 2))
count = 0
ControllerManager.stateReset(ctrlManager)

# Sleep 100 ms. Because ControllerManager need some time to use callback to get the correct states of joints.
rospy.sleep(0.1)


while not rospy.is_shutdown():
    count = count + 1
    try:
        if (count<5):
            manager.display()
        for i in range(numofCamera):
            state[i] = ctrlManager.getState(i)
            points[i] = 20*MarkerManager.spher2cart(state[i])
            # speeds = Optimization.controller(state)
            # ctrlManager.maipulate(speeds,i)
        #display
        MarkerManager.adddisplay(points)

        rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("ROS Interrupt Exception, trying to shut down node")
