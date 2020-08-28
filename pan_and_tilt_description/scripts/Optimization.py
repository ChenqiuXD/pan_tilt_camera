import rospy
from math import pi, sqrt, cos, sin, exp, acos
import numpy as np
import spherical_geometry.polygon as sg
import multiprocessing
from src.pan_and_tilt_description.scripts import MarkerManager

a = np.pi/6
# a represent the max value of angle derivation in the pan/yaw direction
b = 680*np.pi/(6*480)
# b represent the max value of angle derivation in the tilt/pitch direction
radius=20

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
            raise Exception("Parameter is not chosen. Please choose theta or phi ")


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
            raise Exception("Parameter is not chosen. Please choose theta or phi ")


def Perf(q, p):
    # q and p is a vector in spherical coordinate
    if abs(q[0] - p[0]) > b or abs(q[1] - p[1]) > a:
        return Perf1(q, p)
    else:
        return Perf0(q, p)


def PPerf(q, p, thetap=False, phip=False):
    if abs(q[0] - p[0]) > b or abs(q[1] - p[1]) > a:
        return Perf1(q, p, True, thetap, phip)
    else:
        return Perf0(q, p, True, thetap, phip)


def phi(q):
    """q is cart"""
    droneArg = [[20, 0.23, 0.34, 1.0],
                [20.3, 1.5, 0.34, 1.0],
                [20.9, 0, 0, 1.0]]
    droneList = MarkerManager.addDrones(droneArg)
    manager = MarkerManager.MarkerManager(droneList)
    result = manager.calcProb(q)
    return result


def controller(p, v_list, fov_list):
    angle_r_pith = sin(p[0]) * partialH_varphi(p, v_list, fov_list)
    angle_r_yaw = partialH_theta(p, v_list, fov_list)
    angle_r = [angle_r_pith, angle_r_yaw]
    return angle_r


def H(p):
    para = []
    area = 2*pi*radius**2
    N=40000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1/2,1,N)
    pool = multiprocessing.Pool(int(multiprocessing.cpu_count()))
    """Compute the average of function and the area of sphere"""
    for i in range(N):
        randtheta = np.arccos(2 * v - 1)
        randphi = 2 * pi * u
        q = np.array([randtheta[i], randphi[i]])
        para.append((p,q))
    result_N = pool.map(multi_process_H, para)
    pool.close()
    pool.join()
    ave = np.average(result_N)
    result = area*ave
    return result

def H_multiprocess(p, q):
    min_pq = 2*pi
    min_i = None
    """Find the nearest points of P"""
    length = len(p)
    for i in range(length):
        p_cart = MarkerManager.spher2cart(p[i])
        q_cart = MarkerManager.spher2cart(q)
        dis = acos(np.dot(p_cart, q_cart))
        if dis < min_pq:
            min_pq = dis
            min_i = i
    """Compute the performance function and phi"""
    perf = Perf(q, p[min_i])
    q_cart = radius * MarkerManager.spher2cart(q)
    detect_phi = phi(q_cart)
    result = perf * detect_phi
    return result

def multi_process_H(args):
    return H_multiprocess(*args)

def partialH_theta(p, poly_list, arc_list):
    """
    Parameters
    ----------
    p points spherical-coordinate (theta phi)
    list a list of cartesian-coordinates points on the boundary of polygon
    arc_list a list of arcs of FOV
    Returns
    -------
    the partial of objective function
    """
    # monto carlo method to solve the surface integral
    poly_list_sphere = np.empty([len(poly_list), 2])
    for i in range(len(poly_list)):
        poly_list_sphere[i] = MarkerManager.cart2spher(poly_list[i])
    N = 10000
    x_phi = np.random.uniform(min(poly_list_sphere[:, 1]), max(poly_list_sphere[:, 1]), N)
    x_theta = np.random.uniform(min(poly_list_sphere[:, 0]), max(poly_list_sphere[:, 0]), N)
    l_phi = np.random.uniform(min(arc_list[:, 1]), max(arc_list[:, 1]), N)
    poly_list = np.unique(poly_list,axis=0)
    polygon = sg.SingleSphericalPolygon(poly_list)
    area = polygon.area()
    length = 2 * (max(arc_list[:, 1]) - min(arc_list[:, 1]))
    k = Perf0(np.array([max(arc_list[:, 0]), p[1]]), p) - Perf1(np.array([max(arc_list[:, 0]), p[1]]), p)  # f1-f2
    first_term = []
    second_term = []
    para = []
    mint = min(arc_list[:, 0])
    maxt = max(arc_list[:, 0])
    for i in range(N):
        q_l_1 = radius*MarkerManager.spher2cart(np.array([mint,l_phi[i]]))
        q_l_2 = radius*MarkerManager.spher2cart(np.array([maxt,l_phi[i]]))
        temp=(p,x_theta[i],x_phi[i],polygon,q_l_1,q_l_2,True)
        para.append(temp)
    pool = multiprocessing.Pool(int(multiprocessing.cpu_count()/2))
    result = pool.map(multi_process_H_partial, para)
    for i in range(N):
        if result[i][0] is not None:
            first_term.append(result[i][0])
        second_term.append(result[i][1])
    pool.close()
    pool.join()
    if len(first_term) == 0 or len(second_term) == 0:
        print("The length of first term or second term is 0.")
        int_first_term = 0
        int_second_term = 0
    else:
        int_first_term = np.average(first_term) * area
    # line integral - if all sensing boundary lines lie in the spherical polygon
        int_second_term = k * np.average(second_term) * length
    result = int_first_term + int_second_term
    return result

def integral_surface_line(p,x_theta,x_phi,polygon,q_l_1,q_l_2,theta):
    """
    theta is true for theta, and is false for phi
    """
    q_sphere = np.array([x_theta, x_phi])
    q_cart = radius * MarkerManager.spher2cart(q_sphere)
    if polygon.contains_point(q_cart):
        if theta is True:
            first_term = np.array([PPerf(q_sphere, p, thetap=True) * phi(q_cart)]) # question??sin(theta)
        else:
            first_term = np.array([PPerf(q_sphere, p, phip=True) * phi(q_cart)]) # question??sin(theta)
    else:
        first_term = None
    second_term = np.array([-phi(q_l_1), phi(q_l_2)])
    return [first_term, second_term]

def multi_process_H_partial(args):
    return integral_surface_line(*args)

def partialH_varphi(p, poly_list, arc_list):
    """
        Parameters
        ----------
        p points spherical-coordinate (theta phi)
        list a list of cartesian-coordinates points on the boundary of polygon
        arc_list a list of arcs of FOV
        Returns
        -------
        the partial of objective function
        """
    # monto carlo method to solve the surface integral
    poly_list_sphere = np.empty([len(poly_list),2])
    for i in range(len(poly_list)):
        poly_list_sphere[i] = MarkerManager.cart2spher(poly_list[i])
    # N = 1000000
    N = 10000
    x_phi_min = min(poly_list_sphere[:, 1])
    x_phi_max = max(poly_list_sphere[:, 1])
    x_phi = np.random.uniform(x_phi_min, x_phi_max, N)
    x_theta_min = min(poly_list_sphere[:, 0])
    x_theta_max = max(poly_list_sphere[:, 0])
    x_theta = np.random.uniform(x_theta_min, x_theta_max, N)
    l_theta = np.random.uniform(min(arc_list[:, 0]), max(arc_list[:, 0]), N)
    poly_list = np.unique(poly_list,axis=0)
    polygon = sg.SingleSphericalPolygon(poly_list)
    area = polygon.area()
    length = 2 * (max(arc_list[:, 0]) - min(arc_list[:, 0]))
    k = Perf0(np.array([p[0], max(arc_list[:, 1])]), p) - Perf1(np.array([p[0], max(arc_list[:, 1])]), p)  # f1-f2
    para=[]
    minp=min(arc_list[:, 1])
    maxp=max(arc_list[:, 1])
    for i in range(N):
        q_l_1 = radius*MarkerManager.spher2cart(np.array([l_theta[i], minp]))
        q_l_2 = radius*MarkerManager.spher2cart(np.array([l_theta[i], maxp]))
        temp=(p,x_theta[i],x_phi[i],polygon,q_l_1,q_l_2,False)
        para.append(temp)
    pool = multiprocessing.Pool(int(multiprocessing.cpu_count()/2))
    result = pool.map(multi_process_H_partial, para)
    first_term = []
    second_term = []
    for i in range(N):
        if result[i][0] is not None:
            first_term.append(result[i][0])
        second_term.append(result[i][1])
    pool.close()
    pool.join()
    if len(first_term) == 0 or len(second_term) == 0:
        print("The length of first term or second term is 0.")
        x_phi_min = min(poly_list_sphere[:, 1])
        x_phi_max = max(poly_list_sphere[:, 1])
        x_phi = np.random.uniform(x_phi_max, x_phi_min+2*pi, N)
        x_theta_min = min(poly_list_sphere[:, 0])
        x_theta_max = max(poly_list_sphere[:, 0])
        x_theta = np.random.uniform(x_theta_min, x_theta_max, N)
        for i in range(N):
            q_l_1 = radius * MarkerManager.spher2cart(np.array([l_theta[i], minp]))
            q_l_2 = radius * MarkerManager.spher2cart(np.array([l_theta[i], maxp]))
            temp = (p, x_theta[i], x_phi[i], polygon, q_l_1, q_l_2, False)
            para.append(temp)
        pool = multiprocessing.Pool(int(multiprocessing.cpu_count() / 2))
        result = pool.map(multi_process_H_partial, para)
        for i in range(N):
            if result[i][0] is not None:
                first_term.append(result[i][0])
            second_term.append(result[i][1])
        pool.close()
        pool.join()

    int_first_term = np.average(first_term) * area
    # line integral - if all sensing boundary lines lie in the spherical polygon
    int_second_term = k * np.average(second_term) * length
    result = int_first_term + int_second_term
    return result

# bound =
