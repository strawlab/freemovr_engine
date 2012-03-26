#!/usr/bin/env python

from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop
from tornado.web import Application
import tornado.web
from tornado.websocket import WebSocketHandler
import tornado.websocket
import tornado.template

import os
import json

# ROS imports
import roslib; roslib.load_manifest('browser_joystick')
import rospy
from  sensor_msgs.msg import Joy

global joy_pub

class EchoWebSocket(tornado.websocket.WebSocketHandler):
    def open(self):
        print "WebSocket opened"

    def on_message(self, message_raw):
        message = json.loads(message_raw)
        if message['msg']=='lag':
            self.write_message( json.dumps({
                'start':message['start'],
                }))

        if message['msg']=='joy':
            msg = Joy()
            msg.header.stamp = rospy.Time.now()
            msg.axes = message['axes']
            msg.buttons = message['buttons']
            joy_pub.publish(msg)

    def on_close(self):
        print "WebSocket closed"

class MainHandler(tornado.web.RequestHandler):
    def initialize(self, cfg):
        self.cfg = cfg
    def get(self):
        self.render("web_control.html",**self.cfg)

class JSHandler(tornado.web.RequestHandler):
    def initialize(self, cfg):
        self.cfg = cfg
    def get(self):
        self.render("web_control.js",**self.cfg)

settings = dict(
    static_path= os.path.join(os.path.dirname(__file__), "static"),
    cookie_secret=os.urandom(1024),
    template_path=os.path.join(os.path.dirname(__file__), "templates"),
    xsrf_cookies= True,
    )

echo_ws_path = 'echo'
host = '10.0.0.222'
port = 1024
base_url = '%s:%d'%(host,port)

js_path='web_control.js'
dd = {'base_url':base_url,
      'echo_ws_path':echo_ws_path,
      'js_path':js_path,
      }

application = tornado.web.Application([
    (r'/', MainHandler, dict(cfg=dd)),
    (r'/'+js_path, JSHandler, dict(cfg=dd)),
    (r'/'+echo_ws_path, EchoWebSocket),
    ],
                                      **settings)

def main():
    global joy_pub

    url = "http://%s"%base_url
    print "starting web server at", url
    try:
        import qrencode
    except ImportError:
        qrencode = None
    if qrencode is not None:
        _,_,im = qrencode.encode_scaled(url)
        if 1:
            fname = 'link.png'
            im.save(fname)
            print 'URL encoded as a QR code in',fname
        else:
            c = unichr(2588)
            # TODO: print im to console using this unicode block
    else:
        print 'QR encoded link not done'

    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node( node_name, disable_signals=True )

    joy_pub = rospy.Publisher("joy", Joy)

    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(port)
    tornado.ioloop.IOLoop.instance().start()

if __name__ == "__main__":
    main()
