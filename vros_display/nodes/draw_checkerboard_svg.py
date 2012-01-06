#!/usr/bin/env python
from __future__ import division
import argparse

# 6x8 corners
n_rows = 9
n_columns = 7

# The idea is that this will eventually be used to make laser-cut
# chessboards. We will start with black material and the cutouts will
# have white showing through. 

width = 30
height = 30

class Element(object):
    def __init__(self,x0,y0,laser=False):
        self.x0=x0
        self.y0=y0
        self.laser = laser
    def get_rects(self):
        result = []
        if self.laser:
            indent = 1.0
        else:
            indent = 0
        if self.laser:
            opts = dict(stroke = "blue",
                        fill = "none")
            opts['stroke-width'] = "0.1"
        else:
            opts = dict(fill = "white")
        rd = dict(x=self.x0+indent,
                  y=self.y0+indent,
                  width=width-indent,
                  height=height-indent,
                  )
        rd.update(opts)
        return [rd]


def format_tag(tag,opts):
    attrs = []
    for k in opts:
        v = opts[k]
        attrs.append( k+'="'+str(v)+'"')
    attrs = ' '.join(attrs)
    elem = '<'+tag+' '+attrs+'/>\n'
    return elem

def make_svg_elements( rects=None ):
    if rects is None:
        rects = []

    out = ""

    for rect in rects:
        out += format_tag('rect',rect)
    return out

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--laser', help="draw only strokes for lasercutting",
                        default=False, action='store_true')
    args = parser.parse_args()

    total_width = n_columns*width
    total_height = n_rows*height
    print 'total_width',total_width
    print 'total_height',total_height

    laser = args.laser

    if not laser:
        rect0 = dict(x=0,y=0, width=total_width, height=total_height, fill="black")
        rects = [rect0]
    else:
        rects = []

    # elements
    for i in range(n_rows):
        for j in range(n_columns):
            x0 = j*width
            y0 = i*height
            if i%2==1:
                val = 0
            else:
                val = 1
            if j%2==val:
                rects.extend( Element(x0,y0,laser=laser).get_rects() )

    file_contents = """<?xml version="1.0"?>
<svg width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}"
     xmlns="http://www.w3.org/2000/svg" version="1.2" baseProfile="tiny">
{elements}
</svg>
"""

    elements = make_svg_elements( rects=rects )

    s=file_contents.format( w=total_width,
                            h=total_height,
                            elements=elements,
                          )
    with open('checkerboard.svg',mode='w') as fd:
        fd.write(s)
