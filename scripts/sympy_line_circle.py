import sympy

# A line through (ax,ay) in direction (sx,sy) has points (ax+t*sx,
# ay+t*sy). To find the intersection with a circle at the origin with
# radius r, find the points on the line which have distance r from the
# origin. (Equivalently, find the points whose squared distance from
# the origin is r squared.)

ax = sympy.Symbol('ax')
ay = sympy.Symbol('ay')

sx = sympy.Symbol('sx')
sy = sympy.Symbol('sy')

r=sympy.Symbol('r')

t=sympy.Symbol('t')

expr = (ax+sx*t)**2 + (ay+sy*t)**2 - r**2
print sympy.solvers.solve(expr, t)
