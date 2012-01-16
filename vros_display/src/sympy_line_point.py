import sympy
from sympy.utilities.lambdify import lambdastr

# A line through the 3 dimensional vector a in direction s has points
# (ax+t*sx, ay+t*sy, az+t*sz). To find the distance from a point at
# the origin, where the derivative of the distance function (as a
# function of t) has its inflection.

a = sympy.DeferredVector('a')
s = sympy.DeferredVector('x')

t=sympy.Symbol('t')

dist2 = (a[0]+s[0]*t)**2 + (a[1]+s[1]*t)**2 + (a[2]+s[2]*t)**2
ddist2_dt = sympy.diff(dist2,t)
func = sympy.solvers.solve(ddist2_dt, t)
#print func
print lambdastr((a,s),func[0])
