#!/usr/bin/env python
import pickle
import os
import numpy
import argparse
import collections
import numpy as np
import warnings
import scipy.misc

def doit(data,topic_prefix):
    topic_prefixes = None
    displays = []
    for physical_display_id in data.keys():
        for virtual_display_id in data[physical_display_id].keys():
            loaded = data[physical_display_id][virtual_display_id]
            results = loaded['data']
            displays.append( (physical_display_id, virtual_display_id) )

            if topic_prefixes is None:
                topic_prefixes = results.keys()
                topic_prefixes.sort()
            else:
                tmp_topic_prefixes = results.keys()
                tmp_topic_prefixes.sort()
                assert topic_prefixes==tmp_topic_prefixes

    if 1:
        count = None

        for ax in [0,1]:
            for subplot_enum,(physical_display_id, virtual_display_id) in enumerate(displays):
                loaded = data[physical_display_id][virtual_display_id]
                results = loaded['data']
                arr = results[topic_prefix][ax]['address']
                if count is None:
                    # initialize
                    count = np.zeros_like(results[topic_prefix][ax]['address'])
                count += arr >= 0
    scale = 255.0/ np.max( np.max( count ))
    count *= scale
    scipy.misc.imsave( 'valid_%s.png'%topic_prefix, count )

def export_valid_cam_regon_png(fname, topic_prefix):
    fd = open(fname,mode='r')
    data = pickle.load(fd)
    fd.close()

    doit(data,topic_prefix)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str)
    parser.add_argument('camera', type=str)
    args = parser.parse_args()

    export_valid_cam_regon_png(args.filename,args.camera)
