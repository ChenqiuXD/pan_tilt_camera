import rospy
from math import pi, sqrt, cos, sin, tan, exp, acos
import numpy as np
import spherical_geometry.polygon as sg
import multiprocessing
from src.pan_and_tilt_description.scripts import MarkerManager
# import MarkerManager

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
    return acos(np.dot(p_cart, q_cart) / np.dot(p_cart, p_cart))


@jit(nopython=True)
def dist_cart_q(q, p):
    # Calculate the distance when q is in cartesian coordinate and p is in shperical cooridnate
    q_spher = cart2spher(q)
    result = dist(q_spher, p)
    return result


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
    eps1 = 0.07
    # eps2 = eps1 * (pi**2 - dist_max**2)
    eps2 = 0.70
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
        if coef > 0:  # Eliminate the possibility when dist(q,p) > pi/2 (yet u and v could be small after projection onto the plane)
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
    result_T = 0
    for i in range(N):
        randtheta = acos(2 * v[i] - 1)
        randphi = 2 * pi * u[i]
        q = np.array([randtheta, randphi])
        add_elem = H_multiprocess(p, q)
        add_elem_T = H_multiprocess_T(p, q)
        # print("Added element is: ", add_elem)
        result += add_elem
        result_T += add_elem_T
    ave = result * 1.0 / N
    result = area * ave
    ave = result_T * 1.0 / N
    result_T = area * ave
    return result, result_T


@jit(nopython=True)
def H_multiprocess(p, q):
    min_pq = 2 * pi
    min_i = -1
    # Find the nearest points of P
    length = len(p)
    for i in range(length):
        p_cart = spher2cart(p[i])
        q_cart = spher2cart(q)
        dis = acos(np.dot(p_cart, q_cart) / np.dot(p_cart, p_cart))
        if dis < min_pq:
            min_pq = dis
            min_i = i

    # Compute the performance function and phi
    p_min = p[min_i]
    perf = Perf(q, p_min)
    # print("In H_multiprocess, the value is: ", perf)
    q_cart = spher2cart(q)
    detect_phi = phi(q_cart)
    result = perf * detect_phi
    return result


@jit(nopython=True)
def H_multiprocess_T(p, q):
    # Find the maximum of Perf
    length = len(p)
    maxPerf = 0.0
    for i in range(length):
        perf = Perf(q, p[i])
        # print("In H_multiprocess_T, No", i, "th value is: ", perf)
        if perf >= maxPerf:
            maxPerf = perf

    # Compute phi and return result
    q_cart = spher2cart(q)
    detect_phi = phi(q_cart)
    result = maxPerf * detect_phi
    return result


@jit(nopython=True)
def integral_surface(p, x_theta, x_phi, theta):
    """
    Calculating a specific Monte Carlo point surface integral (point : [x_theta, x_phi]).
    theta is true for theta, and is false for phi
    """
    q_sphere = np.array([x_theta, x_phi])
    q_cart = spher2cart(q_sphere)
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
def calcLineIntegral(N, p, isTheta):
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
    M = int(N / 40)
    num_cam = len(p)

    result_l = np.zeros(shape=(num_cam, 1))
    result_lin_long = []
    result_lin_short = []
    # First calculate the length
    tmp = sqrt(1.0 + tan(b) ** 2 + tan(a) ** 2)
    len_1 = 2 * np.arcsin(tan(a) / tmp)   # Longer arc length
    len_2 = 2 * np.arcsin(tan(b) / tmp)   # Shorter arc length

    for j in range(num_cam):
        # Calculate the e_theta and e_phi (p[j,0]->theta(pitch), p[j,1]->phi(yaw))
        e_theta = np.array([cos(p[j, 0]) * cos(p[j, 1]), cos(p[j, 0]) * sin(p[j, 1]), -sin(p[j, 0])])
        e_phi = np.array([-sin(p[j, 1]), cos(p[j, 1]), 0])

        # Calculate the sample points
        tmp1 = np.random.uniform(-pi / 6, pi / 6, M)
        tmp2 = np.random.uniform(-pi / 8, pi / 8, M)
        p_i = spher2cart(p[j])

        # Calculate the normals
        n_down = np.array([cos(p[j, 1]) * cos(p[j, 0] + b), sin(p[j, 1]) * cos(p[j, 0] + b), sin(p[j, 0] + b)])
        n_up = np.array([-cos(p[j, 0] - b) * cos(p[j, 1]), -cos(p[j, 0] - b) * sin(p[j, 1]), sin(p[j, 0] - b)])
        n_left = np.array([
            - sin(p[j, 0]) ** 2 * sin(p[j, 1] + a) - cos(p[j, 0]) ** 2 * sin(p[j, 1]),
            cos(p[j, 0]) ** 2 * cos(p[j, 1]) + sin(p[j, 0]) ** 2 * cos(p[j, 1] + a),
            sin(p[j, 0]) * cos(p[j, 0]) * (sin(p[j, 1]) * cos(p[j, 1] + a) - sin(p[j, 1] + a) * cos(p[j, 1]))
        ])
        n_right = np.array([
            sin(p[j, 0]) ** 2 * sin(p[j, 1] - a) + cos(p[j, 0]) ** 2 * sin(p[j, 1]),
            - cos(p[j, 0]) ** 2 * cos(p[j, 1]) - sin(p[j, 0]) ** 2 * cos(p[j, 1] - a),
            sin(p[j, 0]) * cos(p[j, 0]) * (sin(p[j, 1] - a) * cos(p[j, 1]) - sin(p[j, 1]) * cos(p[j, 1] - a))
        ])

        # Calculate every point
        for i in range(M):
            # Sample four points repectively on the four lines
            q_theta_down = p_i + (tan(b) * e_theta + tan(tmp1[i]) * e_phi) * radius
            q_theta_up = p_i + (tan(-b) * e_theta + tan(tmp1[i]) * e_phi) * radius
            q_phi_left = p_i + (tan(tmp2[i]) * e_theta + tan(a) * e_phi) * radius
            q_phi_right = p_i + (tan(tmp2[i]) * e_theta + tan(-a) * e_phi) * radius
            q_down_sphere = cart2spher(q_theta_down)
            q_down = spher2cart(q_down_sphere)
            q_up_sphere = cart2spher(q_theta_up)
            q_up = spher2cart(q_up_sphere)
            q_left_sphere = cart2spher(q_phi_left)
            q_left = spher2cart(q_left_sphere)
            q_right_sphere = cart2spher(q_phi_right)
            q_right = spher2cart(q_right_sphere)
            if isTheta is True:  # Partial with respect to theta
                # Calculate the partial gamma with respect to theta_i
                # partial_gamma_theta = -sin(d(p_i, gamma))*v_1*p_i + cos(d(p_i, gamma))*e_theta
                dist = dist_cart_q(q_theta_down, p[j])
                parital_gamma_theta_0 = -sin(dist) * tan(pi / 8) * p_i/radius + cos(dist) * e_theta
                dist = dist_cart_q(q_theta_up, p[j])
                parital_gamma_theta_1 = -sin(dist) * tan(-pi / 8) * p_i/radius + cos(dist) * e_theta
                dist = dist_cart_q(q_phi_left, p[j])
                parital_gamma_theta_2 = -sin(dist) * tan(tmp2[i]) * p_i/radius + cos(dist) * e_theta
                dist = dist_cart_q(q_phi_right, p[j])
                parital_gamma_theta_3 = -sin(dist) * tan(tmp2[i]) * p_i/radius + cos(dist) * e_theta
                result_down = np.dot(parital_gamma_theta_0,n_down)*(Perf0(q_down_sphere,p[j])-Perf1(q_down_sphere,p[j]))*phi(q_down)
                result_up = np.dot(parital_gamma_theta_1, n_up) * (
                            Perf0(q_up_sphere, p[j]) - Perf1(q_up_sphere, p[j])) * phi(q_up)
                result_left = np.dot(parital_gamma_theta_2, n_left) * (
                            Perf0(q_left_sphere, p[j]) - Perf1(q_left_sphere, p[j])) * phi(q_left)
                result_right = np.dot(parital_gamma_theta_3, n_right) * (
                            Perf0(q_right_sphere, p[j]) - Perf1(q_right_sphere, p[j])) * phi(q_right)
                result_lin_long.append(result_down)
                result_lin_long.append(result_up)
                result_lin_short.append(result_left)
                result_lin_short.append(result_right)
                # q_l_1 = spher2cart(np.array([mint, l_phi[j][i]]))
                # q_l_2 = spher2cart(np.array([maxt, l_phi[j][i]]))
                # result_lin.append(integral_line_theta(q_l_1, q_l_2))
            else:  # Partial with respect to phi
                temp = np.array([-cos(p[j, 1]), -sin(p[j, 1]), 0])
                # Calculate the partial gamma with respect to varphi_i
                # partial_gamma_theta = -sin(d(p_i, gamma))*v_1*p_i + cos(d(p_i, gamma))*e_theta
                dist = dist_cart_q(q_theta_down, p[j])
                parital_gamma_varphi_0 = sin(dist) * (tan(pi/8)*cos(p[j,0])*e_phi+tan(tmp1[i])*temp) + sin(p[j,0])*cos(dist) * e_phi
                dist = dist_cart_q(q_theta_up, p[j])
                parital_gamma_varphi_1 = sin(dist) * (tan(-pi/8)*cos(p[j,0])*e_phi+tan(tmp1[i])*temp) + sin(p[j,0])*cos(dist) * e_phi
                dist = dist_cart_q(q_phi_left, p[j])
                parital_gamma_varphi_2 = sin(dist) * (tan(tmp2[i])*cos(p[j,0])*e_phi+tan(pi/6)*temp) + sin(p[j,0])*cos(dist) * e_phi
                dist = dist_cart_q(q_phi_right, p[j])
                parital_gamma_varphi_3 = sin(dist) * (tan(tmp2[i])*cos(p[j,0])*e_phi+tan(-pi/6)*temp) + sin(p[j,0])*cos(dist) * e_phi
                result_down = np.dot(parital_gamma_varphi_0, n_down) * (
                            Perf0(q_down_sphere, p[j]) - Perf1(q_down_sphere, p[j])) * phi(q_down)
                result_up = np.dot(parital_gamma_varphi_1, n_up) * (
                        Perf0(q_up_sphere, p[j]) - Perf1(q_up_sphere, p[j])) * phi(q_up)
                result_left = np.dot(parital_gamma_varphi_2, n_left) * (
                        Perf0(q_left_sphere, p[j]) - Perf1(q_left_sphere, p[j])) * phi(q_left)
                result_right = np.dot(parital_gamma_varphi_3, n_right) * (
                        Perf0(q_right_sphere, p[j]) - Perf1(q_right_sphere, p[j])) * phi(q_right)
                result_lin_long.append(result_down)
                result_lin_long.append(result_up)
                result_lin_short.append(result_left)
                result_lin_short.append(result_right)
            result_l[j] = 2*average_1d(result_lin_long)*len_1+2*average_1d(result_lin_short)*len_2
        # k[j] = Perf0(np.array([maxt, p[j, 1]]), p[j]) - Perf1(np.array([maxt, p[j, 1]]), p[j])  # f1-f2

    return result_l


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
    line_s = calcLineIntegral(N, p, isTheta=True)

    result = ave_s * poly_area + line_s
    # result = ave_s * poly_area
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
    line_s = calcLineIntegral(N, p, isTheta=False)
    result = ave_s * poly_area + line_s
    # print("partialH_varphi",line_s, result)
    # result = ave_s * poly_area
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
    angle_r_theta = partialH_theta(p, v_list, fov_list)
    angle_r_phi = a * partialH_varphi(p, v_list, fov_list)
    angle_r = np.append(angle_r_theta, angle_r_phi, axis=1)
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
    result = np.array([x, y, z]) * radius
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
    droneArg = [[RADIUS, 1.3, 0, 1.0],
                [RADIUS, 1.0, 2.1, 1.0],
                [RADIUS, 1.5, 4.1, 1.0]]
    # droneArg = [[RADIUS, 1.3, 0, 1.0]]
    #             # [RADIUS, 2.1, 1.0, 1.0],
    #             # [RADIUS, 4.2, 1.5, 1.0]]
    prob = 0
    coef = [0.05, 0.05, 0.05]
    # coef = [0.003]
    for i in range(len(droneArg)):
        drone = droneArg[i]
        # Calculate distance
        dist = 0.0
        dist += (drone[0] * cos(drone[1]) - q[2]) ** 2
        dist += (drone[0] * sin(drone[1]) * sin(drone[2]) - q[1]) ** 2
        dist += (drone[0] * sin(drone[1]) * cos(drone[2]) - q[0]) ** 2
        dist = sqrt(dist)

        # Add the probability by adding up the probability of each drone occuring at pose q
        prob += drone[3] * np.exp(-coef[i] * dist ** 2)
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

@jit(nopython=True)
def average_1d(array):
    sum = 0.0
    count = 0.0
    for i in array:
        sum += i
        count += 1
    return sum / count

