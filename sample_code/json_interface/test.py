import socket
import time
import json
import random

DS_IP = "127.0.0.1"
DS_PORT = 8888

while 1:
    z = 0.0

    srvsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srvsock.connect((DS_IP, DS_PORT))

    z += (random.random()*0.1)
    j = {"position":{"x":0.0,"y":0.0,"z":z},
         "cylinder_radius":{"data":1+random.random()},
         "cylinder_image":{"data":random.choice(["checkerboard16.png","gray.png"])},
         "foo/bar":{"data":1.2}
    }
    s = json.dumps(j)

    srvsock.sendall(s)

    srvsock.close()

    time.sleep(0.1)
