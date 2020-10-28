import rospy
from math import pi, sqrt, cos, sin, tan, exp, acos
import numpy as np
import spherical_geometry.polygon as sg
import multiprocessing
# from src.pan_and_tilt_description.scripts import MarkerManager
import MarkerManager

# GPU related module
from numba import jit

# a represent the max value of angle derivation in the pan/yaw direction
a = np.pi / 6
# b represent the max value of angle derivation in the tilt/pitch direction
b = np.pi / 6 * (640 / 480)
radius = 100
DIST_THRESH = 5
beta = 0.9
alpha = 0.5


@jit(nopython=True)
def dist(q, p):
    # Calculate the distance between q and p by its angle in between. q and p are 2*1 vector representing [pitch, yaw]
    q_cart = spher2cart(q)
    p_cart = spher2cart(p)
    return acos(np.dot(p_cart, q_cart)/np.dot(p_cart, p_cart))

@jit(nopython=True)
def dist_cart_q(q, p):
    # Calculate the distance when q is in cartesian coordinate and p is in shperical cooridnate
    q_spher = cart2spher(q)
    dist(q_spher, p)

@jit(nopython=True)
def Perf0(q, p, partialp=False, theta=False, fan=False):
    # q and p is a vector in spherical coordinate (inside -a~a and -b~b range)
    if not partialp:
        prob = beta * exp(-alpha * dist(q, p) ** 2)
    else:
        if theta:
            dis = dist(q, p)
            coef = -2 * alpha * beta * exp(-alpha * dis ** 2) * dis

            # Calculate the partial d(p,q) with respect to theta(p)
            partial_dist_theta = calcPartialDist_theta(q, p)

            # Calculate the partial of theta
            prob = coef * partial_dist_theta
        elif fan:
            dis = dist(q, p)
            coef = -2 * alpha * beta * exp(-alpha * dis ** 2) * dis

            # Calculate the partial d(p,q) with respect to fan(p)
            partial_dist_fan = calcPartialDist_fan(q, p)

            # Calculate the partial of theta
            prob = coef * partial_dist_fan
        else:
            # raise Exception("Parameter is not chosen. Please choose theta or phi (from Perf0 function)")
            prob = 0
    return prob


@jit(nopython=True)
def Perf1(q, p, partialp=False, theta=False, fan=False):
    # q and p is a vector in spherical coordinate (outside -a~a and -b~b range)
    # dist_max = 0.617783     # This is the maximum angle distance of pi/6 and pi/8
    # eps1 = beta * exp(-alpha*dist_max**2) / (pi**2-dist_max**2)
    eps1 = 0.007
    # eps2 = eps1 * (pi**2 - dist_max**2)
    eps2 = 0.070
    if not partialp:
        prob = -eps1 * dist(q, p) ** 2 + eps2
    else:
        if theta:
            dis = dist(q, p)
            coef = -2 * eps1 * dis

            # Calculate the partial d(p,q) with respect to theta(p)
            partial_dist_theta = calcPartialDist_theta(q, p)

            # Calculate the partial of theta
            prob = coef * partial_dist_theta
        elif fan:
            dis = dist(q, p)
            coef = -2 * eps1 * dis

            # Calculate the partial d(p,q) with respect to fan(p)
            partial_dist_fan = calcPartialDist_fan(q, p)

            # Calculate the partial of theta
            prob = coef * partial_dist_fan
        else:
            # raise Exception("Parameter is not chosen. Please choose theta or phi (from Perf0 function)")
            prob = 0
    return prob


@jit(nopython=True)
def calcPartialDist_theta(q, p):
    # Calculate partial of dist(q,p) with repect to theta(p)
    product_p_q = cos(q[1]) * sin(q[0]) * cos(p[1]) * sin(p[0]) + sin(q[1]) * sin(q[0]) * sin(p[1]) * sin(p[0]) + cos(
        q[0]) * cos(p[0])
    tmp = -1.0 / sqrt(1 - product_p_q ** 2)
    partial_product_theta = cos(q[1]) * sin(q[0]) * cos(p[1]) * cos(p[0]) + sin(q[1]) * sin(q[0]) * sin(p[1]) * cos(
        p[0]) - cos(q[0]) * sin(p[0])
    partial_dist_theta = tmp * partial_product_theta
    return partial_dist_theta


@jit(nopython=True)
def calcPartialDist_fan(q, p):
    # Calculate the partial d(p,q) with respect to fan(p)
    product_p_q = cos(q[1]) * sin(q[0]) * cos(p[1]) * sin(p[0]) + sin(q[1]) * sin(q[0]) * sin(p[1]) * sin(p[0]) + cos(
        q[0]) * cos(p[0])
    tmp = -1.0 / sqrt(1 - product_p_q ** 2)
    partial_product_fan = cos(q[1]) * sin(q[0]) * (-sin(p[1])) * sin(p[0]) + sin(q[1]) * sin(q[0]) * cos(p[1]) * sin(
        p[0])
    partial_dist_fan = tmp * partial_product_fan
    return partial_dist_fan


@jit(nopython=True)
def checkInsideRange(q, p):
    # Function to check whether q are within the range of p
    cart_q = spher2cart(q)
    cart_p = spher2cart(p)
    
    # Get [x', y', z'] of point q
    coef = np.dot(cart_p, cart_p) / np.dot(cart_p, cart_q)
    posInPlane = coef * cart_q

    # Get the v1 and v2 (where cart_q = v1*e_theta + v2*e_phi)
    e_T = np.array([[cos(p[0]) * cos(p[1]), sin(p[1]) * cos(p[0]), -sin(p[0])], [-sin(p[1]), cos(p[1]), 0]])
    v = np.dot(e_T, posInPlane - cart_p)

    # Check whether v is within range
    result = False
    if (v[0] <= tan(a) * radius and v[0] >= -tan(a) * radius and v[1] <= tan(b) * radius and v[1] >= -tan(b) * radius):
        result = True
    return result


@jit(nopython=True)
def Perf(q, p):
    # q and p is a vector in spherical coordinate
    if checkInsideRange(q, p):
        return Perf0(q, p)
    else:
        return Perf1(q, p)


@jit(nopython=True)
def PPerf(q, p, thetap=False, phip=False):
    if checkInsideRange(q, p):
        return Perf0(q, p, True, thetap, phip)
    else:
        return Perf1(q, p, True, thetap, phip)


@jit(nopython=True)
def H(p):
    area = 2 * pi * radius ** 2
    N = 60000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1 / 2, 1, N)
    # Compute the average of function and the area of sphere
    result = 0
    for i in range(N):
        randtheta = acos(2 * v[i] - 1)
        randphi = 2 * pi * u[i]
        q = np.array([randtheta, randphi])
        add_elem = H_multiprocess(p, q)
        # print("Added element is: ", add_elem)
        result += add_elem
    ave = result * 1.0 / N
    result = area * ave
    return result


@jit(nopython=True)
def H_multiprocess(p, q):
    min_pq = 2 * pi
    min_i = -1
    # Find the nearest points of P
    length = len(p)
    for i in range(length):
        p_cart = spher2cart(p[i])
        q_cart = spher2cart(q)
        dis = acos(np.dot(p_cart, q_cart)/np.dot(p_cart, p_cart))
        if dis < min_pq:
            min_pq = dis
            min_i = i

    # Compute the performance function and phi
    p_min = p[min_i]
    perf = Perf(q, p_min)
    q_cart = spher2cart(q)
    detect_phi = phi(q_cart)
    result = perf * detect_phi
    return result


@jit(nopython=True)
def integral_surface(p, x_theta, x_phi, theta):
    """
    Calculating a specific Monte Carlo point surface integral (point : [x_theta, x_phi]).
    theta is true for theta, and is false for phi
    """
    q_sphere = np.array([x_theta, x_phi])
    q_cart =  spher2cart(q_sphere)
    # Find the nearest points of P
    min_pq = 2 * pi
    min_i = 0
    length = len(p)
    for i in range(length):
        p_cart = spher2cart(p[i])
        dis = acos(np.dot(p_cart, q_cart) / np.dot(p_cart, p_cart))
        if dis < min_pq:
            min_pq = dis
            min_i = i

    if theta is True:
        # first_term = PPerf(q_sphere, p[min_i], thetap=True) * phi(q_cart)
        tmp = PPerf(q_sphere, p[min_i], thetap=True) * phi(q_cart)
        first_term = np.array(tmp)  # question??sin(theta)
    else:
        # first_term = PPerf(q_sphere, p[min_i], phip=True) * phi(q_cart)
        tmp = PPerf(q_sphere, p[min_i], phip=True) * phi(q_cart)
        first_term = np.array(tmp)  # question??sin(theta)

    # second_term = np.array([-phi(q_l_1), phi(q_l_2)])
    return first_term, min_i


@jit(nopython=True)
def integral_line_theta(q_l_1, q_l_2):
    """ Calculate a specific Monte Carlo point line integral """
    return np.array([-phi(q_l_1), phi(q_l_2)])

@jit(nopython=True)
def integral_line_varphi(q_l_1, q_l_2):
    """ Calculate a specific Monte Carlo point line integral """
    return np.array([-phi(q_l_1), phi(q_l_2)])

@jit(nopython=True)
def calcSurfaceIntegral(u, v, N, p, isTheta):
    """
    Parameters
    ----------
    u       :   random numbers from 0-1, len = N   
    v       :   random numbers from 0.5-1, len = N  
    N       :   length of random numbers. Number of Monte Carlo sampling points  
    p       :   each value represents a angle of joint. cam_num*2 matrix  
    isTheta :   True for calculating partial for theta, False for phi  
    Returns
    -------
    Surface integral calculated by Monte Carlo methods with total N sampling points  
    ave_s : float  
    """
    num_cam = len(p)
    count = np.zeros(shape=(num_cam, 1))
    sum = np.zeros(shape=(num_cam, 1))
    for i in range(N):
        randtheta = acos(2 * v[i] - 1)
        randphi = 2 * pi * u[i]
        first_term, min_i = integral_surface(p, randtheta, randphi, isTheta)
        count[min_i] += 1
        sum[min_i] += first_term
    ave_s = sum / count
    return ave_s


@jit(nopython=True)
def calcLineIntegral(u, v, N, p, arc_list, isTheta):
    """
    Parameters
    ----------
    u       :   random numbers from 0-1, len = N   
    v       :   random numbers from 0.5-1, len = N  
    N       :   length of random numbers. Number of Monte Carlo sampling points  
    p       :   each value represents a angle of joint. cam_num*2 matrix  
    arc_list:   the four points of camera in the sphere coordinate (represents in angles). cam_num*4 matrix  
    Returns
    -------
    Line integral calculated by Monte Carlo methods with totally N/4 sampling points  
    ave_l : 4*1 vector  
    length : 4*1 vector   
    k : 4*1 vector  
    """
    M = int(N / 4)
    num_cam = len(p)

    l_phi = np.zeros(shape=(num_cam, M))
    ave_l = np.zeros(shape=(num_cam, 1))
    length = np.zeros(shape=(num_cam, 1))
    k = np.zeros(shape=(num_cam, 1))
    result_lin = []

    # First calculate the length
    tmp = sqrt(1.0 + tan(b)**2 + tan(a)**2)  
    len_1 = 2 * np.arcsin(tan(a) / tmp) * radius  # Longer arc length
    len_2 = 2 * np.arcsin(tan(b) / tmp) * radius  # Shorter arc length

    for j in range(num_cam):
        # Calculate the e_theta and e_phi (p[j,0]->theta(pitch), p[j,1]->phi(yaw))
        e_theta = np.array([cos(p[j, 0])*cos(p[j, 1]), cos(p[j, 0])*sin(p[j, 1]), -sin(p[j, 0])])
        e_phi = np.array([-sin(p[j, 1]), cos(p[j, 1]), 0])

        # Calculate the sample points
        tmp1 = np.random.uniform(-pi/6, pi/6, M)
        tmp2 = np.random.uniform(-pi/8, pi/8, M)
        p_i = spher2cart(p[j])

        # Calculate the normals
        n_up = np.array([ -cos(p[j,0] - b)*cos(p[j,1]), -cos(p[j,0] - b)*sin(p[j,1]), sin(p[j,0]-b) ])
        n_down = np.array([ cos(p[j,1])*cos(p[j,0]+b), sin(p[j,1])*cos(p[j,0+b]), sin(p[j,0]+b) ])
        # n_left = 
        # n_right = 

        # Calculate every point
        for i in range(M):
            if isTheta: # Partial with respect to theta
                # Sample four points repectively on the four lines
                q_theta_max = p_i + ( tan(pi/8) * e_theta + tan(tmp1(i)) * e_phi ) * radius
                q_theta_min = p_i + ( tan(-pi/8) * e_theta + tan(tmp1(i)) * e_phi ) * radius
                q_phi_max = p_i + ( tan(tmp2(i)) * e_theta + tan(pi/6) * e_phi ) * radius
                q_phi_min = p_i + ( tan(tmp2(i)) * e_theta + tan(-pi/6) * e_phi ) * radius

                # Calculate the parital gamma with repect to theta_i
                # partial_gamma_theta = -sin(d(p_i, gamma))*v_1*p_i + cos(d(p_i, gamma))*e_theta
                dist = dist_cart_q(q_theta_max, p[j])
                parital_gamma_theta_0 = -sin(dist)*tan(pi/8)*p_i + cos(dist)*e_theta
                dist = dist_cart_q(q_theta_min, p[j])
                parital_gamma_theta_1 = -sin(dist)*tan(-pi/8)*p_i + cos(dist)*e_theta
                dist = dist_cart_q(q_phi_max, p[j])
                parital_gamma_theta_2 = -sin(dist)*tan(tmp2(i))*p_i + cos(dist)*e_theta
                dist = dist_cart_q(q_phi_min, p[j])
                parital_gamma_theta_3 = -sin(dist)*tan(tmp2(i))*p_i + cos(dist)*e_theta

                # q_l_1 = spher2cart(np.array([mint, l_phi[j][i]]))
                # q_l_2 = spher2cart(np.array([maxt, l_phi[j][i]]))
                # result_lin.append(integral_line_theta(q_l_1, q_l_2))
            else:   # Partial with respect to phi
                # Sample four points
                pass
        # ave_l[j] = average_2d(result_lin)
        # k[j] = Perf0(np.array([maxt, p[j, 1]]), p[j]) - Perf1(np.array([maxt, p[j, 1]]), p[j])  # f1-f2
    
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
    N = 60000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1 / 2, 1, N)
    
    # Calculate poly_area
    num_cam = len(p)
    polygon = []
    poly_area = np.zeros(shape=(num_cam, 1))
    for j in range(num_cam):
        poly_list[j] = np.unique(poly_list[j], axis=0)
        polygon.append(sg.SingleSphericalPolygon(poly_list[j]))
        poly_area[j] = polygon[j].area()

    # surface integral
    ave_s = calcSurfaceIntegral(u, v, N, p, isTheta=True)

    # line integral
    ave_l, length, k = calcLineIntegral(u, v, N, p, arc_list)

    #result = ave_s * poly_area + k * ave_l * length
    result = ave_s * poly_area
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
    result: 4*1 vector  
    """
    # monto carlo method to solve the surface integral
    N = 60000
    u = np.random.uniform(0, 1, N)
    v = np.random.uniform(1 / 2, 1, N)

    # Calculate poly_area
    num_cam = len(p)
    polygon = []
    poly_area = np.zeros(shape=(num_cam, 1))
    for j in range(num_cam):
        poly_list[j] = np.unique(poly_list[j], axis=0)
        polygon.append(sg.SingleSphericalPolygon(poly_list[j]))
        poly_area[j] = polygon[j].area()

    # surface integral
    ave_s = calcSurfaceIntegral(u, v, N, p, isTheta=False)

    # line integral
    ave_l, length, k = calcLineIntegral(u, v, N, p, arc_list)
    
    #result = ave_s * poly_area + k * ave_l * length
    result = ave_s * poly_area
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
    a = np.zeros(shape=(len(p), 1))
    for i in range(len(p)):
        a[i] = 1.0 / sin(p[i, 0])
    angle_r_pitch = a * partialH_varphi(p, v_list, fov_list)
    angle_r_yaw = partialH_theta(p, v_list, fov_list)
    angle_r = np.append(angle_r_pitch, angle_r_yaw, axis=1)
    return angle_r


# Copied from MarkerManager for the use of numba
@jit(nopython=True)
def cart2spher(points):
    """x y z to theta phi"""
    rho = np.sqrt(points[0] ** 2 + points[1] ** 2 + points[2] ** 2)
    theta = acos(points[2] / rho)
    if points[1] >= 0:
        phi = acos(points[0] / np.sqrt(points[0] ** 2 + points[1] ** 2))
    else:
        phi = pi + acos(points[0] / np.sqrt(points[0] ** 2 + points[1] ** 2))
    result = np.array([theta, phi])
    return result


@jit(nopython=True)
def spher2cart(points):
    """theta phi to x y z"""
    z = np.cos(points[0])
    y = np.sin(points[0]) * np.sin(points[1])
    x = np.sin(points[0]) * np.cos(points[1])
    result = np.array([x, y, z])*radius
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
    RADIUS = 100
    droneArg = [[RADIUS, 0, 1.3, 1.0],
                [RADIUS, 0.8, 1.0, 1.0],
                [RADIUS, 1.7, 1.5, 1.0]]
    prob = 0
    for drone in droneArg:
        # Calculate distance
        dist = 0.0
        dist += (drone[0] * sin(drone[2]) - q[2]) ** 2
        dist += (drone[0] * cos(drone[2]) * sin(drone[1]) - q[1]) ** 2
        dist += (drone[0] * cos(drone[2]) * cos(drone[1]) - q[0]) ** 2
        dist = sqrt(dist)

        # Add the probability by adding up the probability of each drone occuring at pose q
        if (dist < DIST_THRESH):
            prob += drone[3] * np.exp(-dist)
        else:
            continue
    return prob


@jit(nopython=True)
def average_2d(array):
    sum = 0.0
    count = 0.0
    for i in array:
        for j in i:
            sum += j
            count += 1
    return sum / count