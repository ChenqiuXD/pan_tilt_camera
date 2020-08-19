import rospy
import numpy as np
from scipy import integrate as ing

a = 0.8
# a represent the max value of angle derivation in the pan direction
b = 0.6


# b represent the max value of angle derivation in the tilt direction

def Perf0(q, p):
    A = np.array([[1, 0], [0, 1]])
    return -(np.linalg.norm(A * (q - p))) ** 2


def Perf1(q, p):
    B = np.array([[1, 0], [0, 1]])
    return -(np.linalg.norm(B * (q - p))) ** 2


def Perf(q, p):
    # q and p is a vector in spherical coordinate
    if abs(max(q[0] - p[0])) > a or abs(max(q[1] - p[1])) > b:
        return Perf1(q, p)
    else:
        return Perf0(q, p)

def controller(p):
    angle_r_pith=[]
    angle_r_yaw=[]
    angle_r = [angle_r_pith, angle_r_yaw]
    return angle_r

def f(x):
    return x + 1


v, err = ing.quad(f, 1, 2)
print(v)
