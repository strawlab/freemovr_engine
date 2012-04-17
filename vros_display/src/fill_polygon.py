import numpy as np
import mahotas.polygon

def posint(x,maxval=np.inf):
    x = int(x)
    if x < 0:
        x = 0
    if x>maxval:
        return maxval
    return x

def fill_polygon(pts,image,fill_value=255):
    if len(pts)>=3:
        height, width = image.shape[:2]
        pts = [ (posint(y,height-1),posint(x,width-1)) for (x,y) in pts]
        mahotas.polygon.fill_polygon(pts, image[:,:,0])
        image[:,:,0] *= fill_value
        image[:,:,1] = image[:,:,0]
        image[:,:,2] = image[:,:,0]
