import rospy
from math import pi, sqrt, cos, sin, exp, acos
import numpy as np
import spherical_geometry.polygon as sg
import multiprocessing
import MarkerManager

# a represent the max value of angle derivation in the pan/yaw direction
a = np.pi/6
# b represent the max value of angle derivation in the tilt/pitch direction
b = 680*np.pi/(6*480)
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


def multi_process_H(args):
    return H_multiprocess(*args)

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


def H(p):
    para = []
    area = 2*pi*radius**2
    N=40000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1/2,1,N)
    pool = multiprocessing.Pool(int(multiprocessing.cpu_count()))
    """Compute the average of function and the area of sphere"""
    for i in range(N):
        randtheta = np.arccos(2 * v[i] - 1)
        randphi = 2 * pi * u[i]
        q = np.array([randtheta, randphi])
        para.append((p,q))
    result_N = pool.map(multi_process_H, para)
    pool.close()
    pool.join()
    ave = np.average(result_N)
    result = area*ave
    return result


def multi_process_H_partial_surface(args):
    return integral_surface(*args)

def multi_process_H_partial_line(args):
    return integral_line(*args)

def integral_surface(p,x_theta,x_phi,polygon,theta):
    """
    theta is true for theta, and is false for phi
    """
    q = np.array([x_theta,x_phi])
    """Find the nearest points of P"""
    min_pq = 2 * pi
    min_i = None
    length = len(p)
    for i in range(length):
        p_cart = radius*MarkerManager.spher2cart(p[i])
        q_cart = radius*MarkerManager.spher2cart(q)
        dis = acos(np.dot(p_cart, q_cart)/(radius**2))
        if dis < min_pq:
            min_pq = dis
            min_i = i

    q_sphere = np.array([x_theta, x_phi])
    q_cart = radius * MarkerManager.spher2cart(q_sphere)

    if theta is True:
        first_term = np.array([PPerf(q_sphere, p[min_i], thetap=True) * phi(q_cart)]) # question??sin(theta)
    else:
        first_term = np.array([PPerf(q_sphere, p[min_i], phip=True) * phi(q_cart)]) # question??sin(theta)

    # second_term = np.array([-phi(q_l_1), phi(q_l_2)])
    return [first_term, min_i]

def integral_line(q_l_1,q_l_2):
    return np.array([-phi(q_l_1), phi(q_l_2)])

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
    N = 40000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1 / 2, 1, N)
    """ camera j"""
    num_cam = len(p)
    polygon=[]
    poly_area = np.zeros(shape=(num_cam,1))
    para=[]
    for j in range(num_cam):
        poly_list[j] = np.unique(poly_list[j],axis=0)
        polygon.append(sg.SingleSphericalPolygon(poly_list[j]))
        poly_area[j]=polygon[j].area()
    """surface integral"""
    for i in range(N):
        randtheta = np.arccos(2 * v[i] - 1)
        randphi = 2 * pi * u[i]
        temp=(p,randtheta,randphi,polygon,True)
        para.append(temp)
    pool = multiprocessing.Pool(int(multiprocessing.cpu_count()/2))
    result_fir = pool.map(multi_process_H_partial_surface, para)
    pool.close()
    pool.join()
    count = np.zeros(shape=(num_cam, 1))
    sum = np.zeros(shape=(num_cam, 1))
    for i in range(len(result_fir)):
        count[result_fir[i][1]] = count[result_fir[i][1]] + 1
        sum[result_fir[i][1]] = sum[result_fir[i][1]] + result_fir[i][0]
    ave_s = sum / count
    """line integral"""
    M=int(N/4)
    l_phi = np.empty([num_cam, M])
    para_l = []
    ave_l = np.zeros(shape=(num_cam, 1))
    length = np.zeros(shape=(num_cam, 1))
    k = np.zeros(shape=(num_cam, 1))
    pool_l = multiprocessing.Pool(int(multiprocessing.cpu_count() / 2))
    for j in range(num_cam):
        mint = min(arc_list[j][:, 0])
        maxt = max(arc_list[j][:, 0])
        length[j] = 2 * (max(arc_list[j][:, 1]) - min(arc_list[j][:, 1]))
        l_phi[j] = np.random.uniform(min(arc_list[j][:, 1]), max(arc_list[j][:, 1]), M)
        for i in range(M):
            q_l_1 = radius * MarkerManager.spher2cart(np.array([mint, l_phi[j][i]]))
            q_l_2 = radius * MarkerManager.spher2cart(np.array([maxt, l_phi[j][i]]))
            temp=(q_l_1,q_l_2)
            para_l.append(temp)
        result_lin = pool_l.map(multi_process_H_partial_line, para_l)
        ave_l[j] = np.average(result_lin)
        k[j] = Perf0(np.array([maxt, p[j,1]]), p[j]) - Perf1(np.array([maxt, p[j,1]]), p[j])  # f1-f2
    pool_l.close()
    pool_l.join()

    result = ave_s*poly_area+k*ave_l*length
    return result

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
    N = 40000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1 / 2, 1, N)
    """ camera j"""
    num_cam = len(p)
    polygon=[]
    poly_area = np.zeros(shape=(num_cam, 1))
    para=[]
    for j in range(num_cam):
        poly_list[j] = np.unique(poly_list[j],axis=0)
        polygon.append(sg.SingleSphericalPolygon(poly_list[j]))
        poly_area[j]=polygon[j].area()
    """surface integral"""
    for i in range(N):
        randtheta = np.arccos(2 * v[i] - 1)
        randphi = 2 * pi * u[i]
        temp=(p,randtheta,randphi,polygon,True)
        para.append(temp)
    pool = multiprocessing.Pool(int(multiprocessing.cpu_count()/2))
    result_fir = pool.map(multi_process_H_partial_surface, para)
    pool.close()
    pool.join()
    count = np.zeros(shape=(num_cam, 1))
    sum = np.zeros(shape=(num_cam, 1))
    for i in range(len(result_fir)):
        count[result_fir[i][1]] = count[result_fir[i][1]] + 1
        sum[result_fir[i][1]] = sum[result_fir[i][1]] + result_fir[i][0]
    ave_s = sum / count
    """line integral"""
    M=int(N/4)
    l_phi = np.empty([num_cam, M])
    para_l = []
    ave_l = np.zeros(shape=(num_cam, 1))
    length = np.zeros(shape=(num_cam, 1))
    k = np.zeros(shape=(num_cam, 1))
    pool_l = multiprocessing.Pool(int(multiprocessing.cpu_count() / 2))
    for j in range(num_cam):
        mint = min(arc_list[j][:, 1])
        maxt = max(arc_list[j][:, 1])
        length[j] = 2 * (max(arc_list[j][:, 0]) - min(arc_list[j][:, 0]))
        l_phi[j] = np.random.uniform(min(arc_list[j][:, 0]), max(arc_list[j][:, 0]), M)
        for i in range(M):
            q_l_1 = radius * MarkerManager.spher2cart(np.array([mint, l_phi[j][i]]))
            q_l_2 = radius * MarkerManager.spher2cart(np.array([maxt, l_phi[j][i]]))
            temp=(q_l_1,q_l_2)
            para_l.append(temp)
        result_lin = pool_l.map(multi_process_H_partial_line,para_l)
        ave_l[j] = np.average(result_lin)
        k[j] = Perf0(np.array([p[j,0], maxt]), p[j]) - Perf1(np.array([p[j,0], maxt]), p[j])  # f1-f2
    pool_l.close()
    pool_l.join()
    result = ave_s*poly_area+k*ave_l*length
    return result

def controller(p, v_list, fov_list):
    a = np.zeros(shape=(len(p),1))
    for i in range(len(p)):
        a[i] = sin(p[i,0])
    angle_r_pith = a * partialH_varphi(p, v_list, fov_list)
    angle_r_yaw = partialH_theta(p, v_list, fov_list)
    angle_r = np.append(angle_r_pith, angle_r_yaw, axis=1)
    return angle_r

