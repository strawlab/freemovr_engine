from __future__ import division
import numpy as np
import imagio
import matplotlib.pyplot as plt

def draw_solidworks_texture():
    '''draw geometry coordinates texture map for use in solidworks'''
    # these constants set the texture size
    WIDTH, HEIGHT = 8192, 2048

    Y,X = np.mgrid[0:HEIGHT, 0:WIDTH]

    red = X/WIDTH
    green = Y/HEIGHT
    blue = np.zeros_like(red)

    image = np.empty( (HEIGHT,WIDTH,3), dtype=np.uint8)
    image[:,:,0] = red*255
    image[:,:,1] = green*255
    image[:,:,2] = blue*255
    imageio.imwrite( 'sw_texcoords_1.png', image )

    im2x_wide = np.repeat( image, 2, 1 )
    imageio.imwrite( 'sw_texcoords_im2x_wide.png', im2x_wide )
    imageio.imwrite( 'sw_texcoords_im2x_left.png',  im2x_wide[:,:WIDTH] )
    imageio.imwrite( 'sw_texcoords_im2x_right.png', im2x_wide[:,WIDTH:] )

    im_narrow = image[:,::2,:]
    im_narrow_2x = np.hstack( (im_narrow,im_narrow) )
    imageio.imwrite( 'sw_im_narrow_2x.png', im_narrow_2x)

def draw_panels():
    WIDTH, HEIGHT = 256, 256
    Y,X = np.mgrid[0:HEIGHT, 0:WIDTH]

    red = X/WIDTH
    green = 1.0-Y/HEIGHT
    blue = np.zeros_like(red)

    image = np.empty( (HEIGHT,WIDTH,3), dtype=np.uint8)
    image[:,:,0] = red*255
    image[:,:,1] = green*255
    image[:,:,2] = blue*255
    imageio.imwrite( 'panels_full.png', image )

    WIDTH, HEIGHT = 256, 20
    Y,X = np.mgrid[0:HEIGHT, 0:WIDTH]

    red = X/WIDTH
    green = np.zeros_like(red)
    blue = np.zeros_like(red)

    image = np.empty( (HEIGHT,WIDTH,3), dtype=np.uint8)
    image[:,:,0] = red*255
    image[:,:,1] = green*255
    image[:,:,2] = blue*255
    imageio.imwrite( 'panels_x.png', image )

    WIDTH, HEIGHT = 20, 256
    Y,X = np.mgrid[0:HEIGHT, 0:WIDTH]

    green = 1.0-Y/HEIGHT
    red = np.zeros_like(green)
    blue = np.zeros_like(green)

    image = np.empty( (HEIGHT,WIDTH,3), dtype=np.uint8)
    image[:,:,0] = red*255
    image[:,:,1] = green*255
    image[:,:,2] = blue*255
    imageio.imwrite( 'panels_y.png', image )

if __name__=='__main__':
    draw_panels()
    draw_solidworks_texture()
