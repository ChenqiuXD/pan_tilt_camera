#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from math import pi, sqrt, cos, sin, exp
import random
from scipy.spatial import SphericalVoronoi, geometric_slerp
import numpy as np

RADIUS = 100


class Drone:
    """ The position and likelihood of drones, alpha is yaw and theta is pitch in a ball coordinate"""
    def __init__(self, length, theta, phi ,probability):
        self.len = length
        self.theta = theta
        self.phi = phi
        self.prob = probability

    def calcDist(self, pose):
        """ Calculate the distance from this drone to the pose """
        if isinstance(pose,np.ndarray):
            dist = 0
            dist += (self.len * cos(self.theta) - pose[2]) ** 2
            dist += (self.len * sin(self.theta) * sin(self.phi) - pose[1]) ** 2
            dist += (self.len * sin(self.theta) * cos(self.phi) - pose[0]) ** 2
            return sqrt(dist)
        elif isinstance(pose, Point):
            dist = 0
            dist += (self.len * cos(self.theta) - pose.z) ** 2
            dist += (self.len * sin(self.theta) * sin(self.phi) - pose.y) ** 2
            dist += (self.len * sin(self.theta) * cos(self.phi) - pose.x) ** 2
            return sqrt(dist)
        else:
            raise Exception("Data type of pose in calDist is wrong.")


class Cameras:
    def __init__(self, numofCamera):
        self.angles = np.zeros([4,2])    # A matrix : cam_num * 2
        self.radius = RADIUS * 0.8   # The range of the cone (length)
        self.field_of_view = 0.174  # in radian, pi/6
        self.num_cam = numofCamera

        # Instantiate the publishers
        self.pubs = [0] * self.num_cam     # Four element, each is a publisher.
        for i in range(self.num_cam):
            topic_name = "camera_range_" + str(i)
            self.pubs[i] = rospy.Publisher(topic_name, Range, queue_size=10)

    def setAngle(self, cam_angle, i):
        # Get the angels from Controller Manager in main.py
        self.angles[i] = cam_angle    

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


class MarkerManager:
    """Manage the half-ball markers to show different possibility of seeing droens. """
    def __init__(self, droneList, numofCamera):
        self.pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker = Marker()
        self.RADIUS = RADIUS
        self.GAP = 0.5     # The distance between two nearby points.
        self.COEF = 100.0   # Used only for range function which only accept int
        self.DIST_THRESH = 20
        self.cameras = Cameras(numofCamera)
        self.drones = droneList

    def setConstantArg(self):
        """ Set some constant args of self.marker """
        self.marker.header.frame_id = "/world"
        self.marker.type = self.marker.POINTS
        self.marker.action = self.marker.ADD
        self.marker.id = 0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2

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
        if len(self.drones):
            prob = self.calcProb(pose)
            if (prob):
                # color.r = self.calcProb(pose) * (1 - colorCoef) + colorCoef
                color.g = 1 - self.calcProb(pose) 
                color.b = 1 - self.calcProb(pose) 
                color.a = 1
            else:
                # color.r = colorCoef
                color.g = 1.0
                color.b = 1.0
                color.a = 0.3
        else:
            color.r = 1.0
            color.a = 0.3
        color.r = 1.0
        # color.b = 1.0
        # color.g = 1.0
        if(color.r <= 0.2):
            print("at pose ", pose.x, " ", pose.y, " ", pose.z, " color.r is: ", color.r)
        return color

    def calcProb(self, pose):
        """ Calculate the probability of drone occuring in pose """
        prob = 0
        coef = [0.02, 0.02, 0.02]
        for i in range(len(self.drones)):
            drone = self.drones[i]
            dist = drone.calcDist(pose)
            prob += drone.prob * exp(-coef[i] * dist ** 2)
            # if dist < self.DIST_THRESH:
                # prob = drone.prob * exp(-dist/self.DIST_THRESH)+prob
        return prob

    def display(self):
        """ Draw a half-ball according to drones list """
        self.setConstantArg()

        # Because function 'range' only accept int, therefore COEF is multiplied.
        heights = (i/self.COEF for i in range(0, int(self.RADIUS*self.COEF), int(self.GAP*self.COEF)))
        # random.seed()
        for h in heights:
            # The radius of the circle of certain height
            radius = sqrt(self.RADIUS ** 2 - h * h)
            perimeter = 2 * pi * radius
            arcLens = (i / self.COEF for i in range(0, int(perimeter * self.COEF), int(self.GAP * self.COEF)))
            for arcLen in arcLens:
                pose = self.calcPose(h, arcLen, perimeter, radius)
                color = self.calcColor(pose)
                # if color.r > 0.3:
                self.marker.colors.append(color)
                self.marker.points.append(pose)
                # else:
                #     continue

        self.pub.publish(self.marker)

    def easy_display(self):
        """ The display function is too time-consuming, therefore a easy_display is adapted which only display
        the red points """
        self.setConstantArg()
        # self.marker.scale.x = 2.0
        for drone in self.drones:
            spher_pos = np.array([drone.theta,drone.phi])
            cart_pos = spher2cart(spher_pos) * drone.len
            pose = Point()
            pose.x = cart_pos[0]
            pose.y = cart_pos[1]
            pose.z = cart_pos[2]
            self.marker.points.append(pose)
            color = ColorRGBA()
            color.r = 1.0
            color.a = 1.0
            self.marker.colors.append(color)
        self.pub.publish(self.marker)


def cart2spher(points):
    """x y z to theta phi"""
    rho = np.sqrt(points[0] ** 2 + points[1] ** 2 + points[2] ** 2)
    theta = np.arccos(points[2]/rho)
    if points[1] >= 0:
        phi = np.arccos(points[0]/np.sqrt(points[0] ** 2 + points[1] ** 2))
    else:
        phi = pi+np.arccos(points[0]/np.sqrt(points[0] ** 2 + points[1] ** 2))
    result = np.array([theta, phi])
    return result

def spher2cart(points):
    """theta phi to x y z"""
    z=np.cos(points[0])
    y=np.sin(points[0])*np.sin(points[1])
    x=np.sin(points[0])*np.cos(points[1])
    result= np.array([x,y,z])
    return result

def addDrones(droneArg):
    """droneList: radius,phi,theta,prob"""
    droneList = []
    if len(droneArg):
        for drone in droneArg:
            droneList.append(Drone(drone[0], drone[1], drone[2], drone[3])) # add drone:radius,theta,phi,prob
    else:
        rospy.logerr("No drone in the droneArg, please recheck your addDrones function")
    return droneList

def addVoronoi(points):
    """add voronoi on the surface. Note that before using this function, modify the parameters(radius, center) first"""
    radius = RADIUS
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
            if not (start == end).all():
                temp=radius*geometric_slerp(start, end, t_vals)
                result = np.concatenate((result, temp[None]), axis=0)
    return result

def addCamerafield(points,delthe,delphi):
    """add the FOV of the camera"""
    radius = RADIUS
    # center = np.array([0, 0, 0])
    t_vals = np.linspace(0, 1, 2000)  # set the num of points on the line
    result = np.random.rand(0, 2000, 3)
    n = len(points)
    for i in range(n-2):
        for j in range(4):
            sphepoints=cart2spher(points[i])
            start = spher2cart(sphepoints+[sqrt(2)*cos(pi/4+j*pi/2)*delthe,sqrt(2)*sin(pi/4+j*pi/2)*delphi])
            end = spher2cart(sphepoints+[sqrt(2)*cos(pi/4+(j+1)*pi/2)*delthe,sqrt(2)*sin(pi/4+(j+1)*pi/2)*delphi])
            temp = geometric_slerp(start, end, t_vals)
            result = np.concatenate((result, temp[None]), axis=0)
    return radius*result

def adddisplay(points):
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.id = 1
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25
    result_v = addVoronoi(points)
    result = result_v
    storepoint=[]
    for i in range(len(result)):
        for j in range(len(result[i])):
            point = Point()
            color = ColorRGBA()
            color.b = 1
            color.g = 1
            color.r = 0
            color.a = 1
            point.x = result[i][j,0]
            point.y = result[i][j,1]
            point.z = result[i][j,2]
            if point.z >= 0:
                marker.points.append(point)
                marker.colors.append(color)
                storepoint.append(point)
    pub.publish(marker)
    return storepoint
