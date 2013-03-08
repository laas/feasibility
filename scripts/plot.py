import numpy as np
from os import walk
#import matplotlib.pylab as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()

files = []
counter=0
for (dirpath, dirname, filenames) in walk("../mcmc/"):
	files.extend(filenames)
	counter=counter+1

for i in range(1,4):
	files.append(files[0])
	counter=counter+1

print files

pN=2
for i in range(0,counter):
	print "go"
	sfile ="../mcmc/"+files[i]
	f=open(sfile, "r")
	lines=f.readlines()
	plt.subplot(pN,pN,i+1)
	x=[]
	y=[]
	z=[]
	d=[]
	for l in lines:
		p=l.split()
		x.append(p[0])
		y.append(p[1])
		z.append(p[2])
		d.append(p[3])


	xx=np.array(x)
	yy=np.array(y)
	zz=np.array(z)
	dd=np.array(d, dtype=np.float64)

	burnin=20
	xx=xx[burnin:-1]
	yy=yy[burnin:-1]
	dd=dd[burnin:-1]

	lim = 1
	p = plt.plot(xx[dd>lim], yy[dd>lim], 'og')
	plt.plot(xx[dd<lim], yy[dd<lim], 'ro')

plt.show()


