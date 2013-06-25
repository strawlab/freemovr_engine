"""
send joystick control as position to fisplay_server

"""

import pygame
import socket
import time
import json
import random
import math

#edsIP = "127.0.0.1"
edsIP = "10.42.99.1"
edsPORT = 8888
pygame.init()
joy = pygame.joystick.Joystick(0)
joy.init()
print 'Initialized Joystick : %s' % joy.get_name()
print 'number of axis = %d' % joy.get_numaxes()
sx = 0.1
sy = 0.1
sz = 5.0
x = 0.0
y = 0.0
t = 0.0
while 1:
    srvsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srvsock.connect((edsIP, edsPORT))
    pygame.event.pump()
    if (joy.get_button(0) == 1):
        x = 0.0
        y = 0.0
    else:
        x = x + sx*joy.get_axis(0)
        y = y + sy*joy.get_axis(1)
        z = sz*joy.get_axis(2)
    
    j = {"position":{"x":x,"y":y,"z":z},"/foo/bar":{"some":"value"}}
    s = json.dumps(j)
    print s
    srvsock.sendall(s)

    srvsock.close()

    time.sleep(0.01)

