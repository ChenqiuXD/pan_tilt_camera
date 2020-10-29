#!/usr/bin/env python3
import numpy as np
from Optimization import Perf
from matplotlib.pyplot import contourf, contour, clabel, show, cm
import matplotlib.pyplot as plt

def test_perf():
    q_0 = np.pi * np.linspace(0,0.5,100)    # Pitch
    q_1 = np.pi * np.linspace(0,1.0,100)    # Yaw
    perf_list = np.zeros(shape=(len(q_0), len(q_0)))
    b = np.array([0, 0])
    for i in range(len(q_0)):
        for j in  range(len(q_1)):
            perf = Perf(np.array([q_0[i],q_1[j]]), b)
            perf_list[i,j] = perf

    q_0_mesh, q_1_mesh = np.meshgrid(q_0, q_1)
    contourf(q_0_mesh, q_1_mesh, perf_list, 8, alpha=0.75, cmap=cm.hot)
    plt.xlabel("yaw")
    plt.ylabel("pitch")
    show()

if __name__ == "__main__":
    test_perf()