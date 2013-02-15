import csv
import collections

import numpy as np
import scipy.misc
import scipy.ndimage.measurements

def get_point_and_lum(arr, thresh, targetr, targetc, errt):
    valid = arr >= thresh

    arr[~valid] = 0

    lbls,npts = scipy.ndimage.measurements.label(arr)
    slcs = scipy.ndimage.measurements.find_objects(lbls)

    #according to the docs, passing None as the last argument should 
    #return all centre of mass. But this doesnt work. So assume the labels start
    #at 1 and increase linearly
    #http://projects.scipy.org/scipy/ticket/1839
    #
    #lets also assume that they are returned in the same order as the slices
    for n in range(1,npts+1):
        slc = slcs[n-1]
        r,c = scipy.ndimage.measurements.center_of_mass(arr,lbls,n)

        errr = abs(targetr-r)
        errc = abs(targetc-c)

        if (errc+errr) < 2:
            b = arr[slc]
            l = b.sum()/np.count_nonzero(b)
            print n, " = ", l,"@",r,",",c
            return r,c,l

    return 0,0,0

if __name__ == "__main__":
    nimgs = 20
    greys = range(0,255+5,5) + [64,127,191]
    thresh = 30
    shutters = (30,40,50,60)

    fieldnames = collections.OrderedDict([('shutter',None),('grayval',None),('point u',None), ('point v',None), ('point l',None)])

    for shutter in shutters:
        with open('%dms.csv' % shutter,'wb') as fou:
            dw = csv.DictWriter(fou, delimiter=',', fieldnames=fieldnames)
            dw.writeheader()
            for g in greys:
                for n in range(nimgs):
                    arr = scipy.misc.imread("%dms/gray%d_%d.png" % (shutter,g,n))
                    r,c,l = get_point_and_lum(arr, thresh, 239, 332, 2)
                    dw.writerow({"shutter":shutter,"grayval":g,"point u":r,"point v":c,"point l":l})

