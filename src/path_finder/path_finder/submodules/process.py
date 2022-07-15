import mapping
import calib
import rrt
import shutil
#opencv,scikit-image-pandas

def process():
	calibration=False
	img_path="3.jpg"
	xstart=703 #Take this point from hoverboard marker
	ystart=246
	#xstart=357
	#ystart=406
	xfin=229
	yfin=315
	try:
	    shutil.rmtree("media")
	except:
	    print("Dir already clean")
	if calibration==True:
	    calib.calib(img_path)
	mapping.mapping(img_path)
	rrt.RRT_call(xstart,ystart,xfin,yfin,"map.jpg")
