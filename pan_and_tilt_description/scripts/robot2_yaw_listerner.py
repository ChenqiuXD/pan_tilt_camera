#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates

def callback(data):
    id = data.name.index("/robot2::pitch_link")
    print data.pose[id].orientation.z, '\n'

if __name__=="__main__":
    rospy.init_node('linkState_listener')
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/gazebo/link_states', LinkStates, callback)
