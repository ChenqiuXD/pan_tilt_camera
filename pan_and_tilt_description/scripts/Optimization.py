import rospy
from math import pi, sqrt, cos, sin, exp, acos
import numpy as np
import spherical_geometry.polygon as sg
import multiprocessing
import MarkerManager

# GPU related module
from numba import jit

# a represent the max value of angle derivation in the pan/yaw direction
a = np.pi/6
# b represent the max value of angle derivation in the tilt/pitch direction
b = np.pi/6*(640/480)
radius=20
DIST_THRESH = 1.5

@jit(nopython=True)
def Perf0(q, p, partialp=False, theta=False, fan=False):
    # q and p is a vector in spherical coordinate
    matrix_a = np.array([[1, 0], [0, 1]])
    if not partialp:
        return -(np.linalg.norm(matrix_a * (q - p), np.inf)) ** 2
    else:
        if theta:
            return 2 * (np.linalg.norm(matrix_a * (q - p), np.inf)) * np.sign(q[1] - p[1])
        elif fan:
            return 2 * (np.linalg.norm(matrix_a * (q - p), np.inf)) * np.sign(q[0] - p[0])
        else:
            # raise Exception("Parameter is not chosen. Please choose theta or phi ")
            return 0

@jit(nopython=True)
def Perf1(q, p, partialp=False, theta=False, fan=False):
    # q and p is a vector in spherical coordinate
    epsilon = 0.2
    matrix_b = epsilon*np.array([[1, 0], [0, 1]])
    if not partialp:
        return -(np.linalg.norm(matrix_b * (q - p), np.inf)) ** 2
    else:
        if theta:
            return 2 * (np.linalg.norm(matrix_b * (q - p), np.inf)) * np.sign(q[1] - p[1])
        elif fan:
            return 2 * (np.linalg.norm(matrix_b * (q - p), np.inf)) * np.sign(q[0] - p[0])
        else:
            # raise Exception("Parameter is not chosen. Please choose theta or phi ")
            return 0

@jit(nopython=True)
def Perf(q, p):
    # q and p is a vector in spherical coordinate
    if abs(q[0] - p[0]) > b or abs(q[1] - p[1]) > a:
        return Perf1(q, p)
    else:
        return Perf0(q, p)

@jit(nopython=True)
def PPerf(q, p, thetap=False, phip=False):
    if abs(q[0] - p[0]) > b or abs(q[1] - p[1]) > a:
        return Perf1(q, p, True, thetap, phip)
    else:
        return Perf0(q, p, True, thetap, phip)

# @jit(nopython=True)
# def H_multiprocess(p, q):
#     min_pq = 2*pi
#     min_i = None
#     # Find the nearest points of P
#     length = len(p)
#     for i in range(length):
#         p_cart = spher2cart(p[i])
#         q_cart = spher2cart(q)
#         dis = acos(np.dot(p_cart, q_cart))
#         if dis < min_pq:
#             min_pq = dis
#             min_i = i
#     # Compute the performance function and phi
#     perf = Perf(q, p[min_i])
#     q_cart = radius * spher2cart(q)
#     detect_phi = phi(q_cart)
#     result = perf * detect_phi
#     return result

# @jit(nopython=True)
# def H(p):
#     area = 2*pi*radius**2
#     N=40000
#     u = np.random.uniform(0, 1, N)
#     v = np.random.uniform(1/2,1,N)
#     # Compute the average of function and the area of sphere
#     result = 0
#     for i in range(N):
#         randtheta = np.arccos(2 * v[i] - 1)
#         randphi = 2 * pi * u[i]
#         q = np.array([randtheta, randphi])
#         result += H_multiprocess(p,q)
#     ave = np.average(result*1.0/N)
#     result = area*ave
#     return result

@jit(nopython=True)
def integral_surface(p,x_theta,x_phi,theta):
    """
    theta is true for theta, and is false for phi
    """
    q_sphere = np.array([x_theta, x_phi])
    q_cart = radius * spher2cart(q_sphere)
    # Find the nearest points of P
    min_pq = 2 * pi
    min_i = 0
    length = len(p)
    for i in range(length):
        p_cart = radius*spher2cart(p[i])
        dis = acos(np.dot(p_cart, q_cart)/(radius**2))
        if dis < min_pq:
            min_pq = dis
            min_i = i

    if theta is True:
        # first_term = PPerf(q_sphere, p[min_i], thetap=True) * phi(q_cart)
        tmp = PPerf(q_sphere, p[min_i], thetap=True) * phi(q_cart)
        first_term = np.array([tmp]) # question??sin(theta)
    else:
        # first_term = PPerf(q_sphere, p[min_i], phip=True) * phi(q_cart)
        tmp = PPerf(q_sphere, p[min_i], phip=True) * phi(q_cart)
        first_term = np.array([tmp]) # question??sin(theta)

    # second_term = np.array([-phi(q_l_1), phi(q_l_2)])
    return [first_term, min_i]

@jit(nopython=True)
def integral_line(q_l_1,q_l_2):
    return np.array([-phi(q_l_1), phi(q_l_2)])

@jit(nopython=True)
def calcSurfaceIntegral(u, v, N, p):
    num_cam = len(p)
    count = np.zeros(shape=(num_cam, 1))
    sum = np.zeros(shape=(num_cam, 1))
    for i in range(N):
        randtheta = np.arccos(2 * v[i] - 1)
        randphi = 2 * pi * u[i]
        first_term, min_i = integral_surface(p,randtheta,randphi,True)
        count[min_i] += 1
        sum[min_i] += first_term
    ave_s = sum / count
    return ave_s

@jit(nopython=True)
def calcLineIntegral(u, v, N, p, arc_list):
    M=int(N/4)
    num_cam = len(p)
    l_phi = np.empty([num_cam, M])
    ave_l = np.zeros(shape=(num_cam, 1))
    length = np.zeros(shape= (num_cam, 1))
    k = np.zeros(shape=(num_cam, 1))
    result_lin = []

    for j in range(num_cam):
        mint = min(arc_list[j][:, 0])
        maxt = max(arc_list[j][:, 0])
        length[j] = 2 * (max(arc_list[j][:, 1]) - min(arc_list[j][:, 1]))
        l_phi[j] = np.random.uniform(min(arc_list[j][:, 1]), max(arc_list[j][:, 1]), M)
        for i in range(M):
            q_l_1 = radius * spher2cart(np.array([mint, l_phi[j][i]]))
            q_l_2 = radius * spher2cart(np.array([maxt, l_phi[j][i]]))
            result_lin.append(integral_line(q_l_1, q_l_2))
        ave_l[j] = np.average(result_lin)
        k[j] = Perf0(np.array([maxt, p[j,1]]), p[j]) - Perf1(np.array([maxt, p[j,1]]), p[j])  # f1-f2
    
    return ave_l, length, k

def partialH_theta(p, poly_list, arc_list):
    """
    Parameters
    ----------
    p       :   points spherical-coordinate (theta phi)
    poly_list:  a list of cartesian-coordinates points on the boundary of polygon
    arc_list:   a list of arcs of FOV
    Returns
    -------
    the partial of objective function
    """
    # monto carlo method to solve the surface integral
    N = 40000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1 / 2, 1, N)
    
    # Calculate poly_area
    num_cam = len(p)
    polygon=[]
    poly_area = np.zeros(shape=(num_cam,1))
    for j in range(num_cam):
        poly_list[j] = np.unique(poly_list[j],axis=0)
        polygon.append(sg.SingleSphericalPolygon(poly_list[j]))
        poly_area[j]=polygon[j].area()

    # surface integral
    ave_s = calcSurfaceIntegral(u, v, N, p)

    # line integral
    ave_l, length, k = calcLineIntegral(u, v, N, p, arc_list)

    result = ave_s*poly_area+k*ave_l*length
    return result

def partialH_varphi(p, poly_list, arc_list):
    """
    Parameters
    ----------
    p       :   points spherical-coordinate (theta phi)
    poly_list:  a list of cartesian-coordinates points on the boundary of voronoi polygon
    arc_list:   a list of arcs of FOV
    Returns
    -------
    the partial of objective function
    """
    # monto carlo method to solve the surface integral
    N = 40000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1 / 2, 1, N)

    # Calculate poly_area
    num_cam = len(p)
    polygon=[]
    poly_area = np.zeros(shape=(num_cam, 1))
    for j in range(num_cam):
        poly_list[j] = np.unique(poly_list[j],axis=0)
        polygon.append(sg.SingleSphericalPolygon(poly_list[j]))
        poly_area[j]=polygon[j].area()

    # surface integral
    ave_s = calcSurfaceIntegral(u, v, N, p)

    # line integral
    ave_l, length, k = calcLineIntegral(u, v, N, p, arc_list)
    
    result = ave_s*poly_area+k*ave_l*length
    return result

def controller(p, v_list, fov_list):
    """
    Generate control command based on state
    Parameters
    ----------
    p       :   states of camera [ [pitch1, yaw1], ... ]
    v_list  :   voronoi space represented by a set of points
    fov_list:   four corners of cameras : [ [upper_left, upper_right, lower_left, lower_right], ... ]
    Returns
    -------
    the control command which is the velocity command of cameras [ [pitch_vel_1, yaw_vel_1], ... ]
    """
    a = np.zeros(shape=(len(p),1))
    for i in range(len(p)):
        a[i] = sin(p[i,0])
    angle_r_pitch = a * partialH_varphi(p, v_list, fov_list)
    angle_r_yaw = partialH_theta(p, v_list, fov_list)
    angle_r = np.append(angle_r_pitch, angle_r_yaw, axis=1)
    return angle_r

# Copied from MarkerManager for the use of numba
@jit(nopython=True)
def cart2spher(points):
    """x y z to theta phi"""
    rho = np.sqrt(points[0] ** 2 + points[1] ** 2 + points[2] ** 2)
    theta = np.arccos(points[2]/rho)
    if points[1] >= 0:
        phi = np.arccos(points[0]/np.sqrt(points[0] ** 2 + points[1] ** 2))+pi/2
    else:
        phi = pi+np.arccos(points[0]/np.sqrt(points[0] ** 2 + points[1] ** 2))+pi/2
    result = np.array([theta, phi])
    return result

@jit(nopython=True)
def spher2cart(points):
    """theta phi to x y z"""
    z=np.cos(points[0])
    y=np.sin(points[0])*np.sin(points[1]-pi/2)
    x=np.sin(points[0])*np.cos(points[1]-pi/2)
    result= np.array([x,y,z])
    return result

@jit(nopython=True)
def phi(q):
    """
    Parameters
    ----------
    q       :   a cartesian-coordinate pose
    Returns
    -------
    the prob of drone occuring at q. With the drone occuring probability already known at certain place
    """
    droneArg = [[20, 0.23, 0.34, 1.0],
                [20.3, 1.5, 0.34, 1.0],
                [20.9, 0, 0, 1.0]]
    prob = 0
    for drone in droneArg:
        # Calculate distance
        dist = 0.0
        dist += (drone[0] * sin(drone[2]) - q[2]) ** 2
        dist += (drone[0] * cos(drone[2]) * sin(drone[1]) - q[2]) ** 2
        dist += (drone[0] * cos(drone[2]) * cos(drone[1]) - q[2]) ** 2
        dist = sqrt(dist)

        # Add the probability by adding up the probability of each drone occuring at pose q
        if (dist<DIST_THRESH):
            prob += drone[3] * np.exp(-dist)
        else:
            continue
    return prob