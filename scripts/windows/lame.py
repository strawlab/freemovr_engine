import socket
import time
import json
import random
import math

#edsIP = "127.0.0.1"
edsIP = "10.42.99.1"
edsPORT = 8888

t = 0.0
while 1:
    srvsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srvsock.connect((edsIP, edsPORT))
	
    t += 0.6
    x = 0.9 * math.sin( t * 0.20 )
    y = 0.9 * math.sin( t * 0.19 )
    z = 0.7 * math.sin( t * 0.05 )
    j = {"position":{"x":x,"y":y,"z":z},"/foo/bar":{"some":"value"}}
    s = json.dumps(j)

    srvsock.sendall(s)

    srvsock.close()

    time.sleep(0.02)
