import sympy

# A line through (ax,ay,az) in direction (sx,sy,sz) has points
# (ax+t*sx, ay+t*sy, az+t*sz). To find the intersection with a sphere
# at the origin with radius r, find the points on the line which have
# distance r from the origin. (Equivalently, find the points whose
# squared distance from the origin is r squared.)

ax = sympy.Symbol('ax')
ay = sympy.Symbol('ay')
az = sympy.Symbol('az')

sx = sympy.Symbol('sx')
sy = sympy.Symbol('sy')
sz = sympy.Symbol('sz')

r=sympy.Symbol('r')

t=sympy.Symbol('t')

expr = (ax+sx*t)**2 + (ay+sy*t)**2 + (az+sz*t)**2 - r**2
print sympy.solvers.solve(expr, t)
