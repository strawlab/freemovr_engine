#!/usr/bin/env python
import sys
import numpy as np
import scipy.ndimage as nd
import matplotlib.pyplot as plt
import yaml

from PIL import Image, ImageDraw

try:
    from ..exr import read_exr, save_exr
except ValueError:
    from exr import read_exr, save_exr

def convexHull (quv):
	hull = scipy.spatial.Delaunay(quv).convex_hull 
	# hull contains now line segments of the convex hull, meaning start and endpoint indices
	# remove double indices (each startpoint is another segments' endpoint'
	ps = set()
	for start_index, end_index in hull:
		ps.add(start_index)
		ps.add(end_index)
	ps = np.array(list(ps)) # ps now contains the indices of the convex hull

	# now sort the vertices of the convex hull clockwise
	center = np.mean(quv[ps], axis=0)
	A = quv[ps] - center
	idx = np.argsort(np.arctan2(A[:,1], A[:,0])) # idx now contains the ordered indices of the convex hull
	return ps[idx]
	
def mergedHull (q1, q2):
	hull1 = scipy.spatial.Delaunay(q1).convex_hull 
	hull2 = scipy.spatial.Delaunay(q2).convex_hull 
	# hull contains now line segments of the convex hull, meaning start and endpoint indices
	# remove double indices (each startpoint is another segments' endpoint'
	ps = set()
	for start_index, end_index in hull1:
		ps.add(start_index)
		ps.add(end_index)
	for start_index, end_index in hull2:
		ps.add(start_index)
		ps.add(end_index)
	ps = np.array(list(ps)) # ps now contains the indices of the convex hull

	# now sort the vertices of the convex hull clockwise
	center = np.mean(q1[ps], axis=0) # TODO: this is wrong, but not much, hopefully:
	# what I really should do is order the two hulls in their respective image spaces and then merge the ordered hulls
	A = q1[ps] - center
	idx = np.argsort(np.arctan2(A[:,1], A[:,0])) # idx now contains the ordered indices of the merged convex hulls
	return ps[idx]
	
def getViewportMask(fname):
	with open(fname, 'r') as f:
		config = yaml.load(f)
		for k in ("display", "p2c", "p2g"):
			if not config.has_key(k):
				print "malformed calibration config, missing %s" % k
				exit()

	virtualDisplays=config["display"]["virtualDisplays"]
	viewports=[]

	for vD in virtualDisplays:
		viewports.append(vD["viewport"])
		
	mask = Image.new('L', (1024, 768), 0)
	drawMask=ImageDraw.Draw(mask)
	color=1;
	for v in viewports:
		a=tuple(tuple(x) for x in v)
		drawMask.polygon(a, fill=color, outline=color) # draw binary mask
		color+=1
	return np.array(mask)
	

def main():   
	in_directory=sys.argv[1] # directory, where input and output files are located 
	savePath=in_directory # path for debug output

	in_file_numbers=[0, 1, 3] # server numbers to put into filenames below
	in_name='display_server%d.nointerp.exr'
	in_name_interp='display_server%d.exr'
	mask_name='ds%d.yaml'

	out_name='display_server%d.blend.exr'

	UV_scale=[2400, 1133] # resolution of intermediary UV map
	#UV_scale=[800, 400]
	images=[]
	masks=[]
	gradients=[]
	fig=plt.figure()
	fig.canvas.set_window_title('Sample Points')
	imgCount=0

	for iarg in range(0,len(in_file_numbers)):
		in_file_name = in_directory + '/' + in_name % in_file_numbers[iarg]
		print "reading: ", in_file_name
		# read OpenEXR file    
		M = read_exr(in_file_name)
		(channels, height, width) = np.shape(M)
		images.append(M)

		mask_fname = in_directory + '/' + mask_name % in_file_numbers[iarg]
		print 'reading: ', mask_fname
		mask_index = getViewportMask(mask_fname)


		
		#import pdb; pdb.set_trace()	
		plt.subplot(3, 1, iarg+1)
		plt.title(in_file_name)
		plt.imshow(mask_index)
		L=M[0]>-0.99 
		YX=np.nonzero(L)
		plt.plot( YX[1], YX[0], ".w")
		for i in [1, 2, 3]: # loop over viewports
			# valid samples are where M[0] > -1 and mask_index==i
			L=np.logical_and(M[0]>-0.99, mask_index==i) 

			# coordinates of valid sample points
			YX=np.nonzero(L)

			# extract valid sample values into vectors
			U=M[0][L]
			V=M[1][L]
			I=M[2][L]

			# test for wrap around of U coordinate (texture seam)
			has_wraparound=((max(U)-min(U))>0.5)
			if has_wraparound:
				L=(U>0.5)
				U[L]-=0.5
				U[np.logical_not(L)]+=0.5

			q=np.transpose(YX)
			quv=np.transpose([V*UV_scale[1], U*UV_scale[0]])
 
			imgCount+=1
			plt.axis('equal')
			plt.plot(np.transpose(q)[1], np.transpose(q)[0], ".")

			ch=mergedHull(q, quv)
			#ch=convexHull(q)
			h = q[ch]
			tp=tuple((x[1], x[0]) for x in h)

			#ch=convexHull(quv)
			huv = quv[ch]
			t=tuple((x[1], x[0]) for x in huv)

			# now generate binary viewport mask in projector space
			mask = Image.new('F', (width, height), 0)
			drawMask=ImageDraw.Draw(mask)
			drawMask.polygon(tp, fill=1) # draw binary mask
			save_exr( savePath+"masks_"+str(imgCount)+".exr", r=mask, g=mask, b=mask, comments='' )		
			masks.append(np.array(mask))

			# now generate binary viewport mask in UV space
			img = Image.new('I', (UV_scale[0], UV_scale[1]), 0)
			draw=ImageDraw.Draw(img)
			draw.polygon(t, fill=1) # draw binary mask
			
			# calculate distance gradient in UV space
			p = np.array(img)
			pg=nd.distance_transform_edt(p)
			#plt.subplot(2,3, 3+i)

			if has_wraparound:
				pg=np.roll(pg, -np.shape(pg)[1]/2, axis=1)

#			save_exr( savePath+"gradient_"+str(imgCount)+".exr", r=pg, g=p, b=pg, comments='' )
			gradients.append(pg) # save gradient to list

	plt.show(block=False)

	# sum over all distance gradients
	gradSum=np.copy(gradients[0])
	for i in range(1, len(gradients)): # loop over other gradients
		gradSum+=gradients[i]
		
	save_exr( savePath+"gradSum.exr", r=gradSum, g=gradSum, b=gradSum, comments='' )	

	# generate blend masks in UV space
	blended=[]

	fig=plt.figure()
	fig.canvas.set_window_title('Blended Viewports in UV')
	for i in range(0,len(gradients)): # loop over viewports
		g=gradients[i]>0
		tr=np.zeros(np.shape(gradients[i])) 
		tr[g]=np.divide(gradients[i][g], gradSum[g])
#		save_exr( savePath+"gradientV_"+str(i)+".exr", r=tr, g=tr, b=tr, comments='' )
		blended.append(tr) # save blend mask to list
		plt.subplot(3, 3, i+1)
		plt.imshow(Image.fromarray(tr*255), origin='lower') # show blended masks
		
	plt.show(block=False)	
	plt.figure()
	count=0	# count masks
	for iarg in range(0,len(in_file_numbers)):
		in_file_name = in_directory + '/' + in_name_interp % in_file_numbers[iarg]
#		print "reading: ", in_file_name
		# read interpolated OpenEXR file    
		M = read_exr(in_file_name)

		# extract channels
		U=(M[0]*UV_scale[0]-0.5).astype(int)
		V=(M[1]*UV_scale[1]-0.5).astype(int)

		# prepare output image
		I=np.zeros(np.shape(U))
		J=np.zeros(np.shape(blended[count]))
		for v in range(1,4): # loop over all viewports of one projector
			#mask=np.nonzero(np.logical_and(masks[count], np.logical_and( M[0]>-1, M[1]>-1)))  # mask contains all pixels of the viewport
			mask=np.nonzero(masks[count])  # mask contains all pixels of the viewport
		#	u=np.max(1, U[mask]);
		#	v=np.max(1, V[mask]);
			# lookup into blended images on cylinder
			I[mask]=blended[count][(V[mask], U[mask])]
			J[(V[mask], U[mask])]+=1
			#I[mask]=gradients[count][(V[mask], U[mask])]+1
			count += 1  # running index of masks

		out_file_name = in_directory + '/' + out_name % in_file_numbers[iarg]
		print "writing: ", out_file_name
		save_exr( out_file_name, r=M[0], g=M[1], b=I, comments='blended masks' )
		#save_exr( savePath+"gradientUV_"+str(iarg)+".exr", r=I, g=I, b=I, comments='blended masks' )
		#save_exr( savePath+"forward_"+str(iarg)+".exr", r=J, g=J, b=J, comments='' )
		
#		import pdb; pdb.set_trace()
			
	# show illustration
		plt.subplot(1, 3, iarg+1)
		plt.imshow(Image.fromarray(I*255), origin='lower') # show blended masks
		#plt.imshow(Image.fromarray(J*50), origin='lower') # show blended masks
		plt.show(block=False)
		
	plt.show()

	
if __name__ == "__main__":
    main()
