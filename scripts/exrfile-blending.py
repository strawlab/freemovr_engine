#!/usr/bin/env python
import roslib
roslib.load_manifest("flyvr")
from flyvr.exr import read_exr, save_exr
from optparse import OptionParser
import os.path
import sys

from flyvr.calibration.blending.cylinder_geometry import cylinder_geometry
from flyvr.calibration.blending.sphere_geometry import sphere_geometry


if __name__ == "__main__":

    parser=OptionParser(usage="usage: %prog [sphere|cylinder] filename(s) [options] filename")
    parser.add_option("-q", "--quiet",
                      action="store_false", dest="verbose", default=True,
                  help="print no info on console")
    parser.add_option("-c", "--concatenate",
                      action="store_true", dest="concatenate", default=False,
                  help="store a single, horizontally concatenated image")
    parser.add_option("-v", "--visualize",
                      action="store_true", dest="visualize", default=False,
                  help="open window with visualization, waits until closed")
    parser.add_option("-m", "--min", dest="minimum", default=0.0, type="float",
                  help="set minimum alpha value for blending", metavar="minimum")

    (options, args) = parser.parse_args()

    if len(args)<2:
        parser.print_usage()
        sys.exit(1)

    geometry_name=args.pop(0).lower()
    if (geometry_name=='sphere'):
        geometry=sphere_geometry()
    elif (geometry_name=='cylinder'):
        geometry=cylinder_geometry()
    else:
        parser.print_usage()
        sys.exit(1)

    images=[]
    for f in args:
        if options.verbose:
            print "Reading file: '%s'"%f
        I=read_exr(f)
        I=np.array(I).transpose(1,2,0)
        images.append(I)

    result=blend(images, geometry, visualize=options.visualize, verbose=options.verbose, concatenate=options.concatenate, minimum=options.minimum)

    # result can be delivered as one large horizontal (in X) matrix for "desktop spanning mode"
    # or as one image per projector
    if options.concatenate:
        path, ext = os.path.splitext(args[0])
        save_exr("%s%s%s"%(path, "_blend_concatenated", ext),
                 result[:,:,0], result[:,:,1], result[:,:,2]) # output one concatenated result image
    else: # split the result in one output image per input image
        for i in range(len(args)):
            path, ext = os.path.splitext(args[i])
            save_exr("%s%s%s"%(path, "_blend", ext),
                     result[i][:,:,0], result[i][:,:,1], result[i][:,:,2]) # output one result image per input image

