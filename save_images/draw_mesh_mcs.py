import numpy as np
from math import exp, log
import matplotlib.pyplot as plt
from scipy.stats import lognorm
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator


def calc_lognorm(x):
    shape = 0.7021940046516504 
    loc = -0.020853993069860244 
    scale = 0.0901587771605952
    y=(x-loc)/scale
    val = (1/y/shape/2.506628)*exp(-(log(y))**2/(2*shape**2))/scale
    return val
    
prev_coords=[208, 208, 40, 0.9]

l=prev_coords[2]
x0=prev_coords[0]
y0=prev_coords[1]
pcs=prev_coords[3]
grid = 0
mu = 1

conf_th_array = np.ones((26,26))
prob_list= np.zeros((26,26))

if l>75:
    grid=13
else: 
    grid=26


for i in range(0,grid):
    xi = i*grid + 0.5*grid
    
    for j in range(0,grid):
        yj = j*grid + 0.5*grid
        
        ddr = ((xi-x0)**2 + (yj-y0)**2)**0.5/l

        prob = calc_lognorm(ddr)
        
        prob_list[i,j]=prob

n = np.sum(prob_list)

conf_th_array = 0.5 - prob_list*mu*pcs/n


print(np.max(conf_th_array), np.min(conf_th_array))
np.save('conf_th_array.npy', conf_th_array)

"""
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

# Make data.
X = np.arange(0, 26, 1)
Y = np.arange(0, 26, 1)
X, Y = np.meshgrid(X, Y)

Z = conf_th_array

# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Customize the z axis.
ax.set_zlim(0.3, 0.5)
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
"""

