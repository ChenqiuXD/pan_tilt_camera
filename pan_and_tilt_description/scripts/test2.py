import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import SphericalVoronoi, geometric_slerp

def addVoronoi(points):
    radius = 1
    center = np.array([0, 0, 0])
    sv = SphericalVoronoi(points, radius, center)
    sv.sort_vertices_of_regions()
    t_vals = np.linspace(0, 1, 2000)
    result = np.random.rand(0, 2000, 3)
    for region in sv.regions:
        n = len(region)
        for i in range(n):
            start = sv.vertices[region][i]
            end = sv.vertices[region][(i + 1) % n]
            temp=geometric_slerp(start, end, t_vals)
            result = np.concatenate((result, temp[None]), axis=0)
    return result
points = np.array([[1, 0, 0], [0, 1, 0], [0, -1, 0], [-1, 0, 0], [0,0,1] ,[0,0,-1]])
print(len(points))
result = addVoronoi(points)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# plot the unit sphere for reference (optional)
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = np.outer(np.cos(u), np.sin(v))
y = np.outer(np.sin(u), np.sin(v))
z = np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x, y, z, color='y', alpha=0.1)
# plot generator points
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b')
# plot Voronoi vertices
ax.scatter(result[...,0,0], result[...,0,1], result[...,0,2],
                   c='g')
for i in range(len(result)):
    ax.plot(result[i][..., 0],
            result[i][..., 1],
            result[i][..., 2],
            c='k')


ax.azim = 10
ax.elev = 40
_ = ax.set_xticks([])
_ = ax.set_yticks([])
_ = ax.set_zticks([])
fig.set_size_inches(4, 4)
plt.show()