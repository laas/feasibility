import numpy as np
import matplotlib.pylab as plt

f=open("../mcmc.tmp", "r");
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

burnin=50
xx=xx[burnin:-1]
yy=yy[burnin:-1]
dd=dd[burnin:-1]

lim = 1
p = plt.plot(xx[dd>lim], yy[dd>lim], 'og')
plt.plot(xx[dd<lim], yy[dd<lim], 'ro')
#plt.setp(p, 'markersize', 2)
plt.show()



