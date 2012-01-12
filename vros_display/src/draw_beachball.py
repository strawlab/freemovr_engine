import numpy as np
import scipy.misc

az = np.linspace(0, 360, 512)
el = np.linspace(-90, 90, 512)

im = np.zeros( (512,512,3), dtype=np.uint8)

im[:, (  0 <= az) & (az <  90), 0] = 0
im[:, ( 90 <= az) & (az < 180), 0] = .333*255
im[:, (180 <= az) & (az < 270), 0] = .666*255
im[:, (270 <= az) & (az < 360), 0] = 1.0*255

im[(-90<=el) & (el<  0),:,1] = 0
im[(  0<=el) & (el< 90),:,1] = .5*255

im[:, ( 90 <= az) & (az < 270), 2] = 1*255

scipy.misc.imsave('beachball.png',im)
