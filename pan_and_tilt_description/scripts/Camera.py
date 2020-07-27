#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

class Camera:
    def __init__(self, robotName, x, y, z):
        topic = robotName + '/joint_states'
        self.sub = rospy.Subscriber(topic, JointState, self.callback)
        self.pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.pitchAng = 0
        self.yawAng = 0
        self.pos = [x, y, z]

    def setConstantArg(self):
        """ Set some constant args of self.marker """
        self.marker.header.frame_id = "/world"
        self.marker.type = self.marker.POINTS
        self.marker.action = self.marker.ADD
        self.marker.id = 1
        self.marker.scale.x = 0.015
        self.marker.scale.y = 0.015
        self.marker.scale.z = 0.015

    def display(self):
        pass

    def callback(self, data):
        self.pitchAng = data.position[0]
        self.yawAng = data.position[1]

if __name__ == "__main__":
    rospy.init_node("listener")
    a = Camera("/robot2", -3, 0, 0.6)   # x, y, z is calculated from main_sim.launch and body.xacro
    while not rospy.is_shutdown():
        try:
            continue
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("ROS Interrupt Exception, trying to shut down node")