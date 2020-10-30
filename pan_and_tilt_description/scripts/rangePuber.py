#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range

class RangePuber:
    def __init__(self, numofCamera, radius):
        self.angles = np.zeros([numofCamera,2])    # A matrix : cam_num * 2
        self.field_of_view = 0.174  # in radian, pi/6
        self.num_cam = numofCamera
        self.radius = radius

        # Instantiate the publishers
        self.pubs = [0] * self.num_cam     # Four element, each is a publisher.
        for i in range(self.num_cam):
            topic_name = "camera_range_" + str(i)
            self.pubs[i] = rospy.Publisher(topic_name, Range, queue_size=10)

    def pub_range(self):
        rang_msg = Range()
        for i in range(self.num_cam):
            rang_msg.header.stamp = rospy.Time.now()
            frame_id = "robot"+str(i+1)+"/camera_link"
            rang_msg.header.frame_id = frame_id
            rang_msg.field_of_view =  self.field_of_view # 0-ULTRASOUND, 1-INFRARED
            rang_msg.min_range = 0.1    # min_range of sonar, useless here
            rang_msg.max_range = 201    # max_range of sonar, useless here
            rang_msg.range = self.radius
            self.pubs[i].publish(rang_msg)

if __name__ == "__main__":
    rospy.init_node("rangePub")
    a = RangePuber(4, 100)
    while not rospy.is_shutdown():
        try:
            a.pub_range()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("ROS Interrupt Exception, trying to shut down node")
        rospy.sleep(0.3)

