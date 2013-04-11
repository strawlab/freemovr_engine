#!/usr/bin/env python
import sys
import flyvr.exr
import numpy as np
import scipy.cluster.hierarchy
from scipy import ndimage
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
   
savePath='/mnt/hgfs/VMware_shared/'
in_directory=sys.argv[1] # directory, where input and output files are located 

in_file_numbers=[0, 1, 3] # server numbers to put into filenames below
in_name='display_server%d.nointerp.exr'
in_name_interp='display_server%d.exr'
out_name='display_server%d.blend.exr'

UV_scale=[2400, 1133] # resolution of intermediary UV map
#UV_scale=[400, 200]
images=[]
masks=[]
gradients=[]
fig=plt.figure()
fig.canvas.set_window_title('Sample Points')
imgCount=0

for iarg in range(0,3):
	in_file_name = in_directory + '/' + in_name % in_file_numbers[iarg]
	print "reading: ", in_file_name
	# read OpenEXR file    
	M = flyvr.exr.read_exr(in_file_name)
	(channels, height, width) = np.shape(M)
	images.append(M)
	# valid samples are where M[0] > -1
	L=M[0]>-0.99

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

	# ToDo: change this to viewport masks from file
	cluster= scipy.cluster.hierarchy.fclusterdata(np.transpose(YX), 3.0, criterion='maxclust', metric='euclidean', depth=1, method='single')

	xy=np.transpose(YX)
	uv=np.transpose([V*UV_scale[1], U*UV_scale[0]])
	
	plt.subplot(3, 1, iarg)
	plt.title(in_file_name)
	for i in [1, 2, 3]: # loop over viewports

		# draw viewport samples
		q=xy[cluster==i]
		imgCount+=1
		plt.axis('equal')
		plt.plot(np.transpose(q)[1], np.transpose(q)[0], ".")

		quv=uv[cluster==i]
		hull = scipy.spatial.Delaunay(q).convex_hull
		# now sort the convex hull clockwise
		ps = set()
		for x, y in hull:
			ps.add(x)
			ps.add(y)
		ps = np.array(list(ps)) # ps now contains the indices of the convex hull
		
		# convex hull in projector space
		center = np.mean(q[ps], axis=0)
		A = q[ps] - center
		idx = np.argsort(np.arctan2(A[:,1], A[:,0])) # idx now contains the ordered indices of the convex hull
		h = q[ps[idx]]
		tp=tuple((x[1], x[0]) for x in h)

		# convex hull projected to UV space (not convex anymore!)
		center = np.mean(quv[ps], axis=0)
		A = quv[ps] - center
		idx = np.argsort(np.arctan2(A[:,1], A[:,0])) # idx now contains the ordered indices of the convex hull
		huv = quv[ps[idx]]
		t=tuple((x[1], x[0]) for x in huv)

		# now generate binary viewport mask in projector space
		mask = Image.new('L', (width, height), 0)
		drawMask=ImageDraw.Draw(mask)
		drawMask.polygon(tp, fill=1) # draw binary mask
		masks.append(np.array(mask))

		# now generate binary viewport mask in UV space
		img = Image.new('L', (UV_scale[0], UV_scale[1]), 0)
		draw=ImageDraw.Draw(img)
		draw.polygon(t, fill=1) # draw binary mask
		
		# calculate distance gradient in UV space
		p = np.array(img)
		pg=ndimage.distance_transform_edt(p) 
		#plt.subplot(2,3, 3+i)

		if has_wraparound:
			pg=np.roll(pg, -np.shape(pg)[1]/2, axis=1)

		# show & file
		flyvr.exr.save_exr( savePath+"gradient_"+str(i)+".exr", r=pg, g=pg, b=pg, comments='' )
		gradients.append(pg) # save gradient to list

plt.show(block=False)

#fig=plt.figure()
#fig.canvas.set_window_title('Viewport Masks in Projector Space')
#position=0;
#for m in masks:
	#position += 1
	#plt.subplot(3, 3, position)
	#plt.imshow(m)

#fig=plt.figure()
#fig.canvas.set_window_title('Summed Distance Transforms in UV')
# sum over all distance gradients
gradSum=np.copy(gradients[0])
for i in range(1, len(gradients)): # loop over other gradients
	gradSum+=gradients[i]
	
#plt.imshow(Image.fromarray(gradSum*10), origin='lower')
#flyvr.exr.save_exr( savePath+"gradient_sum.exr", r=gradSum, g=gradSum, b=gradSum, comments='' )	

# generate blend masks in UV space
blended=[]

fig=plt.figure()
fig.canvas.set_window_title('Blended Viewports in UV')
for i in range(0,len(gradients)): # loop over viewports
	g=gradients[i]>0
	tr=np.zeros(np.shape(gradients[i])) 
	tr[g]=np.divide(gradients[i][g], gradSum[g])
	flyvr.exr.save_exr( savePath+"gradientV_"+str(i)+".exr", r=tr, g=tr, b=tr, comments='' )
	blended.append(tr) # save blend mask to list
	plt.subplot(3, 3, i+1)
	plt.imshow(Image.fromarray(tr*255), origin='lower') # show blended masks
	
plt.show(block=False)	
plt.figure()
count=0	# count masks
for iarg in range(0,3):
	in_file_name = in_directory + '/' + in_name_interp % in_file_numbers[iarg]
	print "reading: ", in_file_name
	# read interpolated OpenEXR file    
	M = flyvr.exr.read_exr(in_file_name)

	# extract channels
	U=(M[0]*UV_scale[0]).astype(int)
	V=(M[1]*UV_scale[1]).astype(int)

	# prepare output image
	I=np.zeros(np.shape(U))

	for v in range(1,4): # loop over all viewports of one projector
		print count
		mask=np.nonzero(np.logical_and(masks[count], np.logical_and( M[0]>-1, M[1]>-1)))  # mask contains all pixels of the viewport
		#import pdb; if (count==8): pdb.set_trace()
		#u=max(1, round(U(mask)));
		#v=max(1, round(V(mask)));
		
		# lookup into blended images on cylinder
		I[mask]=blended[count][(V[mask], U[mask])]
		count += 1  # running index of masks
		
# show illustration
	plt.subplot(1, 3, iarg+1)
	plt.imshow(Image.fromarray(I*255), origin='lower') # show blended masks
	plt.show(block=False)
	
plt.show()

	
