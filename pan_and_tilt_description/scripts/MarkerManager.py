#!/usr/bin/env python
import rospy
from Camera import Camera
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from math import pi, sqrt, cos, sin, exp
from scipy.spatial import SphericalVoronoi, geometric_slerp
import random
import numpy as np


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
        dist += (self.len * sin(self.theta) - pose.z) ** 2
        dist += (self.len * cos(self.theta) * sin(self.alpha) - pose.y) ** 2
        dist += (self.len * cos(self.theta) * cos(self.alpha) - pose.x) ** 2
        return sqrt(dist)


class MarkerManager:
    """Manage the half-ball markers to show different possibility of seeing droens. """
    def __init__(self, droneList):
        self.pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker = Marker()
        self.RADIUS = 20
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
        angle = arcLen / perimeter * 2 * pi
        pose.x = radius * cos(angle)
        pose.y = radius * sin(angle)
        pose.z = h
        return pose

    def calcColor(self, pose):
        """ Calculate the color according to the pose with respect to drones """
        color = ColorRGBA()
        colorCoef = 0.3
        if len(self.drones):
            prob = self.calcProb(pose)
            if (prob):
                color.r = self.calcProb(pose) * (1 - colorCoef) + colorCoef
            else:
                color.r = colorCoef
        else:
            color.r = colorCoef
        color.b = 0.3
        color.g = 0.3
        color.a = 1
        return color

    def calcProb(self, pose):
        """ Calculate the probability of drone occuring here """
        prob = 0
        for drone in self.drones:
            dist = drone.calcDist(pose)
            if dist < self.DIST_THRESH:
                prob += drone.prob * exp(-dist)
        return prob

    def display(self):
        """ Draw a half-ball according to drones list """
        self.setConstantArg()
        heights = (i / self.COEF for i in range(0, int(self.RADIUS * self.COEF), int(self.GAP * self.COEF)))
        # random.seed()
        for h in heights:
            # The radius of the circle of certain height
            radius = sqrt(self.RADIUS * self.RADIUS - h * h)
            perimeter = 2 * pi * radius
            arcLens = (i / self.COEF for i in range(0, int(perimeter * self.COEF), int(self.GAP * self.COEF)))
            for arcLen in arcLens:
                pose = self.calcPose(h, arcLen, perimeter, radius)
                self.marker.points.append(pose)
                color = self.calcColor(pose)
                self.marker.colors.append(color)

        self.pub.publish(self.marker)

def cart2spher(points):
    rho = np.sqrt(points[0] ** 2 + points[1] ** 2 + points[2] ** 2)
    theta = np.arccos(points[2]/rho)
    phi = np.arccos(points[0]/np.sqrt(points[0] ** 2 + points[1] ** 2))
    result = np.array([theta, phi])
    return result

def spher2cart(points):
    z=np.cos(points[0])
    y=np.sin(points[0])*np.sin(points[1])
    x=np.sin(points[0])*np.cos(points[1])
    result= np.array([x,y,z])
    return result

def addDrones(droneArg):
    droneList = []
    if len(droneArg):
        for drone in droneArg:
            droneList.append(Drone(drone[0], drone[1], drone[2], drone[3]))#add drone:radius,yaw,theta,prob
    else:
        rospy.logerr("No drone in the droneArg, please recheck your main function")
    return droneList


def addVoronoi(points):
    """add voronoi on the surface. Note that before using this function, modify the parameters(radius, center) first"""
    radius = 20
    points = points/radius
    center = np.array([0, 0, 0])
    sv = SphericalVoronoi(points, 1, center)
    sv.sort_vertices_of_regions()
    t_vals = np.linspace(0, 1, 2000)#set the num of points on the line
    result = np.random.rand(0, 2000, 3)
    for region in sv.regions:
        n = len(region)
        for i in range(n):
            start = sv.vertices[region][i]
            end = sv.vertices[region][(i + 1) % n]
            temp=radius*geometric_slerp(start, end, t_vals)
            result = np.concatenate((result, temp[None]), axis=0)
    return result

def addCamerafield(points,delthe,delphi):
    """add the FOV of the camera"""
    radius = 1
    center = np.array([0, 0, 0])
    t_vals = np.linspace(0, 1, 2000)  # set the num of points on the line
    result = np.random.rand(0, 2000, 3)
    n = len(points)
    for i in range(n):
        for j in 4:
            sphepoints=cart2spher(points[i])
            start = spher2cart(sphepoints+sqrt(2)*[cos(pi/4+j*pi/2)*delthe,sin(pi/4+j*pi/2)*delphi])
            end = spher2cart(sphepoints+sqrt(2)*[cos(pi/4+(j+1)*pi/2)*delthe,sin(pi/4+(j+1)*pi/2)*delphi])
            temp = geometric_slerp(start, end, t_vals)
            result = np.concatenate((result, temp[None]), axis=0)
    return result

def adddisplay(points):
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    point = Point()
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.id = 1
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    result_v = addVoronoi(points)
    color = ColorRGBA()
    color.b = 1
    color.g = 1
    color.r = 0
    color.a = 0.5
    for i in range(len(result_v)):
        for j in range(len(result_v[i])):
            print(result_v[i][j])
            point.x = result_v[i][j,0]
            point.y = result_v[i][j,1]
            point.z = result_v[i][j,2]
            marker.points.append(point)
            marker.colors.append(color)
    pub.publish(marker)

if __name__ == "__main__":
    droneArg = [[20, 0.23, 0.34, 1],
                [20.3, 1.5, 0.34, 1],
                [20.9, 0, 0, 1]]
    points = np.array([[20, 0, 0], [0, 20, 0], [0, -20, 0], [-20, 0, 0], [0, 0, 20], [0, 0, -20]])
    droneList = addDrones(droneArg)
    manager = MarkerManager(droneList)
    rospy.init_node('markerManager')
    rate = rospy.Rate(1)
    print("Starting publishing markers")
    count=0
    while not rospy.is_shutdown():
        count = count+1
        try:
            if (count == 1):
                manager.display()
                adddisplay(points)
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("ROS Interrupt Exception, trying to shut down node")
