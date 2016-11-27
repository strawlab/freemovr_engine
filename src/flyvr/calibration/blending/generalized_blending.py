#!/usr/bin/env python
import roslib
roslib.load_manifest("flyvr")
import matplotlib.pyplot as plt
from flyvr.exr import read_exr, save_exr
import numpy as np
import cv2
import os.path
import sys

def blend(images, geometry, UV_scale=[1000, 1000], visualize=False, verbose=False, concatenate=False, minimum=0.0):
    """calculate blending masks for overlapping projector beam paths

    Args:
        images: List of input images which map XY projector pixels to UV coordinates of the screen geometry
                Have to be of the same size and contain 3 channels:
                channel 0: U coordinate
                channel 1: V coordinate
                channel 2: sometimes contains measured intensity, is ignored

        geometry: Object to calculate distance function on screen geometry, e.g. see "cylinder_geometry.py"

    Named Args:
        UV_scale:    resolution of intermediary UV map (magic number depending on geometry and input resolution)
                     ToDo: determine this automatically. As it is, please experiment.
        visualize:   When True, visualizes input images and blend mask interactively in a window. 
                     waits until window is closed manually.
        verbose:     produces additional information on the console when True

        concatenate: When True, concatenates the result images along their X dimensions.

    Returns:
        If "concatenate==False", returns a list of output images, Channel 0 and 1 are the same as in the input, channel 2 contains blend factor (0..1)
        If "concatenate==True", returns a concatenation of the above images in a single image

    """

    start=True
    beampaths_count=0
    contours=[]
    if (len(images) < 1 ):
        raise Exception("Need at least one input image")

    input_height, input_width, input_channels =images[0].shape

    # read & concatenate input maps, determine & label beampaths
    for I in images:
        h,w,ch = I.shape
        I[:,:,2]=np.zeros(np.shape(I[:,:,2]))
        I=np.nan_to_num(I)
        if (h != input_height) or (w != input_width) or (ch != input_channels):
            raise Exception("Input images must be the same size")

        L=np.zeros(np.shape(I[:,:,0]), np.uint8)
        Lpadded=np.zeros(np.add(np.shape(I[:,:,0]), [2, 2]), np.uint8) # pad with 1px border=0
        L[I[:,:,0]>-0.99]=1 # L contains 1s where beampath is valid
        cv2.copyMakeBorder(L, 1, 1, 1, 1, cv2.BORDER_CONSTANT, Lpadded, 0)
        cnts,_ = cv2.findContours(Lpadded,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_NONE, offset=(-1,-1)) # offset compensates padding
        for c in cnts:
            contours.append(c)
            beampaths_count+=1
            color=beampaths_count
            cv2.drawContours(L,[c],0,color,-1) # fill contour with individual color
        # append new data on the right of image
        if start:
            M=I
            Label=L
            start=False
        else:
            offset_u=np.shape(M)[1]
            for cnt in cnts:
                cnt[:,0,0]=np.add(cnt[:,0,0], offset_u)
            M=(np.concatenate((M,I), axis=1))
            Label=(np.concatenate((Label,L), axis=1))

    if verbose:
        print "%d contour(s) found."%beampaths_count

    # now we got all the input maps concatenated in "M"
    # and labeled beam paths in "Label"

    # plot a nice overview image
    if visualize:
        plt.figure()
        plt.imshow(M)
        fig = plt.gcf()
        fig.canvas.set_window_title('Input maps')
        plt.figure()
        plt.imshow(Label)
        fig = plt.gcf()
        fig.canvas.set_window_title('Beam path classification')

        plt.show(block=False)

    # extract valid sample values into vectors
    L=M[:,:,0]>-0.99
    U=M[:,:,0][L]
    V=M[:,:,1][L]
    I=M[:,:,2][L] # channel 2 sometimes contains measured intensity, is ignored

    # now determine the distance functions for each beam path
    distance=np.zeros([np.shape(contours)[0], np.shape(M)[0], np.shape(M)[1]])
    grad_sum=np.zeros(np.shape(M[:,:,0]))
    gi=0
    for cnt in contours:
        # ui,vi contain indices of boundary pixels
        ui=cnt[:,0,0]
        vi=cnt[:,0,1]
        # Ub,Vb contain UV coordinates of boundary pixels
        Ub=M[vi,ui,0]
        Vb=M[vi,ui,1]
        # calculate distance in image space for selected screen geometry
        distance[gi][L]=geometry.calculate_distance(Ub, Vb, UV_scale, U, V)
        grad_sum=np.add(grad_sum, distance[gi]) # sum over all distances in image space
        gi+=1

    # calculate the blending masks for all beam paths    
    K=np.zeros(np.shape(M[:,:,0]))
    for i in range(beampaths_count):
        mask=np.logical_and(Label==i+1, grad_sum>0.0) # binary mask for this beampath

        # calculate blending mask for this beam path:
        #  for each pixel inside the beam: distance[beampath]/grad_sum
        # this results in a smooth gradient in the areas overlapping other
        # beam paths in UV and 1 everywhere else inside the beam 
        K[mask] = np.clip(np.divide(distance[i][mask], grad_sum[mask]) + minimum, 0.0, 1.0)

    if visualize:
        plt.figure()
        plt.imshow(K)
        plt.set_cmap('gray')
        fig = plt.gcf()
        fig.canvas.set_window_title('Blending masks')
        plt.show(block=True)

    # result can be delivered as one large horizontal (in X) matrix for "desktop spanning mode"
    # or as one image per projector
    result=np.dstack((M[:,:,0],M[:,:,1],K))
    if concatenate:
        return result # output one concatenated result image
    else: # split the result in one output image per input image
        return np.split(result, len(images), axis=1)

