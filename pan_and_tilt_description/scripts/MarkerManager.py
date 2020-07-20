#!/usr/bin/env python
import rospy
from Camera import Camera
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from math import pi, sqrt, cos, sin, exp
import random

class Drone:
    """ The position and likelihood of drones, alpha is yaw and theta is pitch in a ball coordinate"""
    def __init__(self, length, alpha, theta, probability):
        self.len = length
        self.alpha = alpha
        self.theta = theta
        self.prob = probability
    
    def calcDist(self, pose):
        """ Calculate the distance from this drone to the pose """
        dist = 0
        dist += (self.len*sin(self.theta) - pose.z) ** 2
        dist += (self.len*cos(self.theta)*sin(self.alpha) - pose.y) ** 2
        dist += (self.len*cos(self.theta)*cos(self.alpha) - pose.x) ** 2
        return sqrt(dist)

class MarkerManager:
    """Manage the half-ball markers to show different values of points."""
    def __init__(self, droneList):
        self.pub = rospy.Publisher("visualization_marker_array", Marker, queue_size=10)
        self.marker = Marker()
        self.RADIUS = 5
        self.GAP = 0.05
        self.COEF = 100.0
        self.DIST_THRESH = 1.5
        self.cameras = []
        self.drones = droneList

    def setConstantArg(self):
        """ Set some constant args of self.marker """
        self.marker.header.frame_id = "/world"
        self.marker.type = self.marker.POINTS
        self.marker.action = self.marker.ADD
        self.marker.id = 0
        self.marker.scale.x = 0.015
        self.marker.scale.y = 0.015
        self.marker.scale.z = 0.015

    def calcPose(self, h, arcLen, perimeter, radius):
        pose = Point()
        angle = arcLen/perimeter*2*pi
        pose.x = radius*cos(angle)
        pose.y = radius*sin(angle)
        pose.z = h
        return pose

    def calcColor(self, pose):
        """ Calculate the color according to the pose with respect to drones """
        color = ColorRGBA()
        if len(self.drones):
            color.r = self.calcProb(pose)
        else:
            color.r = 0.0        
        color.g = 0.1
        color.b = 0.3
        color.a = 1
        return color
    
    def calcProb(self, pose):
        """ Calculate the probability of drone occuring here """
        id, dist = self.findNearestDrone(pose)
        prob = 0
        if dist < self.DIST_THRESH:
            drone = self.drones[id]
            prob = drone.prob * exp(-dist)
        return prob

    def findNearestDrone(self, pose):
        minDist = 100
        id = 0
        minid = 0
        for drone in self.drones:
            dist = drone.calcDist(pose)
            if dist<minDist:
                minDist = dist
                minid = id
            id += 1
        return minid, minDist

    def display(self):
        """ Draw a half-ball according to drones list """
        self.setConstantArg()
        heights = (i/self.COEF for i in range(0, int(self.RADIUS*self.COEF), int(self.GAP*self.COEF)))
        # random.seed()
        for h in heights:
            # The radius of the circle of certain height
            radius = sqrt(self.RADIUS*self.RADIUS - h*h)
            perimeter = 2*pi*radius
            arcLens = (i/self.COEF for i in range(0, int(perimeter*self.COEF), int(self.GAP*self.COEF)))
            for arcLen in arcLens:
                pose = self.calcPose(h, arcLen, perimeter, radius)
                self.marker.points.append(pose)
                color = self.calcColor(pose)
                self.marker.colors.append(color)

        self.pub.publish(self.marker)

def addDrones(droneArg):
    droneList = []
    if len(droneArg):
        for drone in droneArg:
            droneList.append(Drone(drone[0], drone[1], drone[2], drone[3]))
    else:
        rospy.logerr("No drone is added, please recheck your main function")
    return droneList

if __name__ == "__main__":
    droneArg = [ [5, 0.23, 0.34, 1],
                 [4.3, 1.5, 0.34, 1],
                 [5.5, 0, 0, 1] ]
    droneList = addDrones(droneArg)
    manager = MarkerManager(droneList)
    rospy.init_node('markerManager')
    rate = rospy.Rate(10)
    print("Starting publishing marekers")
    while not rospy.is_shutdown():
        try:
            manager.display()
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("ROS Interrupt Exception, trying to shut down node")