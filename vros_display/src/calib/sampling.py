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

if __name__ == "__main__":
    print tuple(gen_horiz_snake(15,10,3,2))
    print
    print tuple(gen_horiz_snake(15,10,3))
    print
    print tuple(gen_horiz_snake(15,10,3,2,linspace=True))
    print 
    print tuple(gen_vert_snake(15,10,3))
