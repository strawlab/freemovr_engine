import sympy

# A line through (ax,ay,az) in direction (sx,sy,sz) has points
# (ax+t*sx, ay+t*sy, az+t*sz). To find the distance from a point at
# the origin, where the derivative of the distance function (as a
# function of t) has its inflection.

ax = sympy.Symbol('ax')
ay = sympy.Symbol('ay')
az = sympy.Symbol('az')

sx = sympy.Symbol('sx')
sy = sympy.Symbol('sy')
sz = sympy.Symbol('sz')

t=sympy.Symbol('t')

dist2 = (ax+sx*t)**2 + (ay+sy*t)**2 + (az+sz*t)**2
ddist2_dt = sympy.diff(dist2,t)
print sympy.solvers.solve(ddist2_dt, t)
