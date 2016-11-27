#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import argparse
import numpy as np

import roslib; roslib.load_manifest('flyvr')
from flyvr.exr import save_exr

def gen_exr(fname=None, width=None, height=None, luminance=None):
    u = np.linspace( 0.0, 1.0, width )
    v = np.linspace( 0.0, 1.0, height )

    U,V = np.meshgrid(u,v)
    B = luminance*np.ones_like(U)
    save_exr( fname, r=U, g=V, b=B)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--width', type=int, default=1920,
                        help="width of display (in pixels)")
    parser.add_argument('--height', type=int, default=1080,
                        help="width of display (in pixels)")
    parser.add_argument('--fname', type=str, default='monitor.exr',
                        help="name of generated EXR file")
    parser.add_argument('--luminance', type=float, default=1.0,
                        help="luminance value to write into 3rd channel")
    args = parser.parse_args()

    gen_exr( fname=args.fname, width=args.width, height=args.height, luminance=args.luminance)

