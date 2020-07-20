#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates

link_name = "/robot2::yaw_link"

rospy.init_node('linkState_listener')
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    
    rate.sleep()
