import numpy as np

def gen_horiz_snake(w,h,sw,sh=None,linspace=False,startw=0,starth=0):
    sh = sw if sh == None else sh
    if linspace:
        fniter = np.linspace
    else:
        fniter = range
        
    reverse = 0
    for hh in fniter(*(starth,h,sh)):
        if reverse:
            for ww in reversed(fniter(*(startw,w,sw))):
                yield ww,hh
            reverse = 0
        else:
            for ww in fniter(*(startw,w,sw)):
                yield ww,hh
            reverse = 1

def gen_vert_snake(w,h,sw,sh=None,linspace=False,startw=0,starth=0):
    sh = sw if sh == None else sh
    if linspace:
        fniter = np.linspace
    else:
        fniter = range
        
    reverse = 0
    for ww in fniter(*(startw,w,sw)):
        if reverse:
            for hh in reversed(fniter(*(starth,h,sh))):
                yield ww,hh
            reverse = 0
        else:
            for hh in fniter(*(starth,h,sh)):
                yield ww,hh
            reverse = 1

def gen_spiral_snake(w, h, sw, sh=None, startw=0, starth=0):
    sh = sw if sh == None else sh
    x,y = 0,0   
    dx, dy = 0, -1

    w *= int((1.0/sw))
    h *= int((1.0/sh))

    for dumb in range(w*h):
        if abs(x) == abs(y) and [dx,dy] != [1,0] or x>0 and y == 1-x:  
            dx, dy = -dy, dx            # corner, change direction

        if abs(x)>w/2 or abs(y)>h/2:    # non-square
            dx, dy = -dy, dx            # change direction
            x, y = -y+dx, x+dy          # jump

        yield (x*sw)+startw, (y*sh)+starth
        x, y = x+dx, y+dy

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    def do_plt(gen):
        plt.figure()
        x,y = [],[]
        for _x,_y in gen:
            x.append(_x)
            y.append(_y)
        print(len(x))
        plt.plot(x,y)
        #plt.xlim(-180,180)
        #plt.ylim(-50,50)

    do_plt(gen_horiz_snake(15,10,3,2))
    do_plt(gen_horiz_snake(15,10,1,1,startw=-2,starth=-4))
    do_plt(gen_horiz_snake(15,10,3,2,linspace=True))
    do_plt(gen_vert_snake(15,10,3))
    do_plt(gen_spiral_snake(12,5,1))
    do_plt(gen_spiral_snake(5,5,1,startw=5,starth=5))
    do_plt(gen_spiral_snake(3,3,0.5,startw=0,starth=0))
    do_plt(gen_spiral_snake(3,3,0.5,startw=-2,starth=2))
    do_plt(gen_spiral_snake(3,3,0.5,startw=-2,starth=-2))
    do_plt(gen_spiral_snake(6,6,1,startw=-86.755104064941406,starth=-39.857143402099609))
    do_plt(gen_spiral_snake(6,6,0.5,startw=-86.755104064941406,starth=-39.857143402099609))

    plt.show()
