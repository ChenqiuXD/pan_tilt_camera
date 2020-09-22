#!/usr/bin/env python

import rospy,tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point
import os.path

def addHeatmaps(droneList):
    # Building height. Plz see main_sim.launch for detail
    HEIGHT = 116.8

    print("Waiting for gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    print("Get it")

    # Load heatmap model from path
    my_path = os.path.abspath(os.path.dirname(__file__))
    path = os.path.join(my_path, "../models/heatmap/model.sdf")
    with open(path, "r") as f:
        product_xml = f.read()

    # Add heatm ap for every drone
    count = 0
    for drone in droneList:
        # drone: [roll, pitch, yaw]
        qua = tf.transformations.quaternion_from_euler(0,drone[0],drone[1])
        orient = Quaternion(qua[0], qua[1], qua[2], qua[3])
        item_name = "heatmap_" + str(count)
        count = count + 1
        print ("Spawning model:s", item_name)
        # Note that every heatmap should be positioned at origin and only change orient 
        item_pose = Pose(Point(x=0, y=0, z=HEIGHT), orient)
        spawn_model(item_name, product_xml, "", item_pose, "world")


if __name__ == '__main__':
    rospy.init_node("spawn_heatmap")
    droneList = [ [1.57, 0], [0,0], [0, -1.57] ]
    addHeatmaps(droneList)
