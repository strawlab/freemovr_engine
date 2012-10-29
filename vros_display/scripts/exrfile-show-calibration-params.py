#!/usr/bin/env python

import os.path
import argparse
import OpenEXR

class ExrShow:
    def __init__(self, path):
        exr = OpenEXR.InputFile(path)
        comments = exr.header()['comments']
        print "%s\n\t%s" % (path,comments)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
    '--exr', type=str, action='append', required=True, help=\
    "display calibration exr file")

    args = parser.parse_args()

    for e in args.exr:
        ExrShow(os.path.expanduser(e))
