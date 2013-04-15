#!/usr/bin/env python
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
import yaml


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
	return mask


	
in_name='../data/Feb22/ds%d.yaml'
plt.figure()
count=1
for number in [0,1,3]:
	fname=in_name % number
	print fname
	plt.subplot(1,3,count)
	count+=1
	mask=getViewportMask(fname)
	import pdb; pdb.set_trace()	
	plt.imshow(mask)
plt.show()
	
