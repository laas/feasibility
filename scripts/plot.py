import numpy as np
import matplotlib.pylab as plt

f=open("../build/mcmc.tmp", "r");
lines=f.readlines();

x=[]
y=[]
d=[]
for l in lines:
	p=l.split()
	x.append(p[0])
	y.append(p[1])
	d.append(p[2])


xx=np.array(x)
yy=np.array(y)
dd=np.array(d, dtype=np.float64)


p = plt.plot(xx[dd>0], yy[dd>0], 'og')
plt.plot(xx[dd==0], yy[dd==0], 'ro')
#plt.setp(p, 'markersize', 2)
plt.show()



