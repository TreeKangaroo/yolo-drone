import numpy as np
from math import exp, log
import matplotlib.pyplot as plt
from scipy.stats import lognorm

def calc_lognorm(x, shape, loc, scale):


    y=(x-loc)/scale
    val = (1/y/shape/2.506628)*exp(-(log(y))**2/(2*shape**2))/scale
    return val

data=np.load('dist_dif.npy')
data=np.delete(data, 862)
data_ind=np.argsort(data)

print(data[data_ind[-3:]])
print(data_ind[-1])

xspace=np.linspace(0,0.8,100)

s1, loc1, scale1 = lognorm.fit(data, loc=0)
print('s: ', s1, 'loc: ', loc1, 'scale: ', scale1)

pdf_fitted = lognorm.pdf(xspace, s1, loc=loc1, scale=scale1)
pdf_calculated=[]
for i in xspace:
    pdf_calculated.append(calc_lognorm(i, s1, loc1, scale1))

fig,ax = plt.subplots(1,1)
ax.hist(data, bins=100, density=True)
ax.plot(xspace, pdf_calculated, 'g-')
ax.plot(xspace, pdf_fitted, 'r-')


ax.set_xlim(0,0.8)
plt.show()



