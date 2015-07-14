#!/usr/bin/env python
"""This script takes a fmf video recorded with fview
!!! MAKE SURE YOU TICKED 'USE HOST TIMESTAMPS' IN FVIEW !!!
and prints the delay for each frame.

REQUIRES:
    sudo apt-get install zbar-tools

"""
import motmot.FlyMovieFormat.FlyMovieFormat as FlyMovieFormat
import sys
import os
import pylab
import numpy
import subprocess
import Image
import motmot.imops.imops as imops


def main():
    try:
        filename = sys.argv[1]
    except:
        print 'Usage: %s fmf_filename' % sys.argv[0]
        sys.exit()


    path,ext = os.path.splitext(filename)
    if ext != '.fmf':
        print 'fmf_filename does not end in .fmf'
        sys.exit()

    fly_movie = FlyMovieFormat.FlyMovie(filename)
    n_frames = fly_movie.get_n_frames()
    fmf_format = fly_movie.get_format()

    fmf = fly_movie
    delays = numpy.array([])
    for frame_number in range(n_frames):
        frame,timestamp = fmf.get_frame(frame_number)

        mono=False
        if (fmf_format in ['RGB8','ARGB8','YUV411','YUV422'] or
            fmf_format.startswith('MONO8:') or
            fmf_format.startswith('MONO32f:')):
            save_frame = imops.to_rgb8(fmf_format, frame)
        else:
            if fmf_format not in ['MONO8','MONO16']:
                warnings.warn('converting unknown fmf format %s to mono'%(
                    fmf_format,))
            save_frame = imops.to_mono8(fmf_format,frame)
            mono=True
        h, w = save_frame.shape[:2]
        if mono:
            im = Image.fromstring('L',(w,h),save_frame.tostring())
        else:
            im = Image.fromstring('RGB',(w,h),save_frame.tostring())
        f = '%s_%08d.%s'%(os.path.join("./", "zbartmp"), frame_number, 'bmp')
        im.save(f)
        try:
            TS = subprocess.check_output(['zbarimg', '-q', f])
            ts = float(TS[8:].strip())
        except OSError:
            raise
        except:
            ts = float('nan')
        delay = timestamp-ts
        delays = numpy.append(delays,[delay])
        print "ds: % 14.6f cam: % 14.6f delay: % 8.6f" % (ts, timestamp, delay)
        os.unlink(f)
    print "delay mean: % 8.6f std: % 8.6f" % (delays[~numpy.isnan(delays)].mean(),delays[~numpy.isnan(delays)].std())
    print "delay max: % 8.6f min: % 8.6f" % (delays[~numpy.isnan(delays)].max(),delays[~numpy.isnan(delays)].min())
    print "%i of %i frames used" % (len(delays[numpy.isnan(delays)]),len(delays[~numpy.isnan(delays)]))
if __name__=='__main__':
    main()
