import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import roslib
roslib.load_manifest('flyvr')
from simple_geom import Cylinder, Vec3

def vec3(a,b,c):
    return dict(x=a, y=b, z=c)

if 1:
    cyl = Cylinder(base=vec3(0,0,0), axis=vec3(0,0,1), radius=0.5)
    tc0 = []
    tc1 = []
    n = 32
    twopi = 2*np.pi
    dt = twopi/n

    def xfrac(idx):
        theta = (idx*dt)%twopi
        frac = theta/twopi
        return frac

    for i in range(n+1):
        tc0.append( xfrac(i) ); tc1.append( 1.0 )
        tc0.append( xfrac(i) ); tc1.append( 0.0 )
        tc0.append( xfrac(i+1) );  tc1.append( 0.0 )
        tc0.append( xfrac(i) );  tc1.append( 1.0 )
    tc = np.array( (tc0, tc1) ).T
    wc = cyl.texcoord2worldcoord(tc)

    if 1:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot( wc[:,0],
                 wc[:,1],
                 wc[:,2], )
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
    fig.savefig('cyl.svg')
