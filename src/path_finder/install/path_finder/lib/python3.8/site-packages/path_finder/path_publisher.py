import numpy as np
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cv2
import numpy as np
import math
import random

import os
import pandas as pd
from skimage.measure import label, regionprops, regionprops_table
from skimage.io import imread, imshow
import matplotlib.pyplot as plt
import shutil


def RRT_call(xstart,ystart,xfinish,yfinish,map):
    class Nodes:
        """Class to store the RRT graph"""
        def __init__(self, x,y):
            self.x = x
            self.y = y
            self.parent_x = []
            self.parent_y = []

    # check collision
    def collision(x1,y1,x2,y2):
        color=[]
        x = list(np.arange(x1,x2,(x2-x1)/100))
        y = list(((y2-y1)/(x2-x1))*(x-x1) + y1)
        print("collision",x,y)
        for i in range(len(x)):
            print(int(x[i]),int(y[i]))
            color.append(img[int(y[i]),int(x[i])])
        if (0 in color):
            return True #collision
        else:
            return False #no-collision

    # check the  collision with obstacle and trim
    def check_collision(x1,y1,x2,y2):
        _,theta = dist_and_angle(x2,y2,x1,y1)
        x=x2 + stepSize*np.cos(theta)
        y=y2 + stepSize*np.sin(theta)
        print(x2,y2,x1,y1)
        print("theta",theta)
        print("check_collision",x,y)

        # TODO: trim the branch if its going out of image area
        # print("Image shape",img.shape)
        hy,hx=img.shape
        if y<0 or y>hy or x<0 or x>hx:
            print("Point out of image bound")
            directCon = False
            nodeCon = False
        else:
            # check direct connection
            if collision(x,y,end[0],end[1]):
                directCon = False
            else:
                directCon=True

            # check connection between two nodes
            if collision(x,y,x2,y2):
                nodeCon = False
            else:
                nodeCon = True

        return(x,y,directCon,nodeCon)

    # return dist and angle b/w new point and nearest node
    def dist_and_angle(x1,y1,x2,y2):
        dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
        angle = math.atan2(y2-y1, x2-x1)
        return(dist,angle)

    # return the neaerst node index
    def nearest_node(x,y):
        temp_dist=[]
        for i in range(len(node_list)):
            dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
            temp_dist.append(dist)
        return temp_dist.index(min(temp_dist))

    # generate a random point in the image space
    def rnd_point(h,l):
        new_y = random.randint(0, h)
        new_x = random.randint(0, l)
        return (new_x,new_y)


    def RRT(img, img2, start, end, stepSize):
        h,l= img.shape # dim of the loaded image
        # print(img.shape) # (384, 683)
        # print(h,l)

        # insert the starting point in the node class
        # node_list = [0] # list to store all the node points         
        node_list[0] = Nodes(start[0],start[1])
        node_list[0].parent_x.append(start[0])
        node_list[0].parent_y.append(start[1])

        # display start and end
        cv2.circle(img2, (start[0],start[1]), 5,(0,0,255),thickness=3, lineType=8)
        cv2.circle(img2, (end[0],end[1]), 5,(0,0,255),thickness=3, lineType=8)

        i=1
        pathFound = False
        while pathFound==False:
            nx,ny = rnd_point(h,l)
            print("Random points:",nx,ny)

            nearest_ind = nearest_node(nx,ny)
            nearest_x = node_list[nearest_ind].x
            nearest_y = node_list[nearest_ind].y
            print("Nearest node coordinates:",nearest_x,nearest_y)

            #check direct connection
            tx,ty,directCon,nodeCon = check_collision(nx,ny,nearest_x,nearest_y)
            print("Check collision:",tx,ty,directCon,nodeCon)

            if directCon and nodeCon:
                print("Node can connect directly with end")
                node_list.append(i)
                node_list[i] = Nodes(tx,ty)
                node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
                node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
                node_list[i].parent_x.append(tx)
                node_list[i].parent_y.append(ty)

                cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
                cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
                cv2.line(img2, (int(tx),int(ty)), (end[0],end[1]), (255,0,0), thickness=2, lineType=8)

                print("Path has been found")
                #print("parent_x",node_list[i].parent_x)
                path_found=[]
                for j in range(len(node_list[i].parent_x)-1):
                    cv2.line(img2, (int(node_list[i].parent_x[j]),int(node_list[i].parent_y[j])), (int(node_list[i].parent_x[j+1]),int(node_list[i].parent_y[j+1])), (255,0,0), thickness=2, lineType=8)
                    path_found.append([int(node_list[i].parent_x[j+1]),int(node_list[i].parent_y[j+1])])
                path_found.append([xfinish, yfinish])
                np.save("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/found_path.npy",path_found)
                # cv2.waitKey(1)
                #cv2.imwrite("media/"+str(i)+".jpg",img2)
                cv2.imwrite("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/out.jpg",img2)
                break

            elif nodeCon:
                print("Nodes connected")
                node_list.append(i)
                node_list[i] = Nodes(tx,ty)
                node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
                node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
                # print(i)
                # print(node_list[nearest_ind].parent_y)
                node_list[i].parent_x.append(tx)
                node_list[i].parent_y.append(ty)
                i=i+1
                # display
                #cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
                #cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
                #cv2.imwrite("media/"+str(i)+".jpg",img2)
                #cv2.imshow("sdc",img2)
                #cv2.waitKey(1)
                continue

            else:
                print("No direct con. and no node con. :( Generating new rnd numbers")
                continue

    def draw_circle(event,x,y,flags,param):
        global coordinates
        if event == cv2.EVENT_LBUTTONDBLCLK:
            cv2.circle(img2,(x,y),5,(255,0,0),-1)
            coordinates.append(x)
            coordinates.append(y)



        # remove previously stored data
    try:
        os.system("rm -rf media")
    except:
        print("Dir already clean")
    os.mkdir("media")
    start=[xstart,ystart]
    stop=[xfinish,yfinish]
    img = cv2.imread(map,0) # load grayscale maze image
    img2 = cv2.imread(map) # load colored maze image
    start = tuple(start) #(20,20) # starting coordinate
    end = tuple(stop) #(450,250) # target coordinate
    stepSize = 120 # stepsize for RRT
    node_list = [0] # list to store all the node points

    coordinates=[]

    # run the RRT algorithm 
    RRT(img, img2, start, end, stepSize)

def mapping(image_path):
    image=cv2.imread(image_path)
    img_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    imgor=image.copy()
    ret,thresh = cv2.threshold(img_grey,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    edges = cv2.Canny(image,50,110)
    kernel = np.ones((3,3),np.uint8)
    edges= cv2.dilate(edges,kernel,iterations = 2)
    image_fin=cv2.bitwise_xor(thresh,edges)
    image_fin = cv2.erode(image_fin, kernel, iterations=2)
    label_im = label(image_fin)
    regions = regionprops(label_im)
    properties = ["area","convex_area","bbox_area", "extent",  
                "mean_intensity", "solidity", "eccentricity", 
                "orientation"]
    pd.DataFrame(regionprops_table(label_im, img_grey, 
                properties=properties))

    masks = []
    bbox = []
    list_of_index = []
    for num, x in enumerate(regions):
        area = x.area
        convex_area = x.convex_area
        if (num!=0 and (area>10)):
            masks.append(regions[num].convex_image)
            bbox.append(regions[num].bbox)   
            list_of_index.append(num)
    count = len(masks)

    rgb_mask = np.zeros_like(label_im)
    for x in list_of_index:
        rgb_mask += (label_im==x+1).astype(int)
    red  =  imgor[:,:,0] * rgb_mask
    green = imgor[:,:,1] * rgb_mask
    blue  = imgor[:,:,2] * rgb_mask
    image = np.dstack([red, green, blue])
    masks_new=[]
    bbox_new=[]
    list_of_index_new = []
    labels_new=[]
    #cv2.imwrite("try.jpg",image)
    #x=cv2.imread("try.jpg")
    """
    for b in bbox:
        cv2.rectangle(x, (b[0], b[1]), (b[2], b[3]), (255,0,0), 2)
    """
    """
    cv2.imshow("try",x)
    def click_event(event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, ' ', y)
            for num, r in enumerate(regions):
                if x>r.bbox[0] and x<r.bbox[2] and y>r.bbox[1] and y<r.bbox[3]:
                    masks_new.append(regions[num].convex_image)
                    bbox_new.append(regions[num].bbox)   
                    list_of_index_new.append(num)
            if label_im[x][y] not in labels_new:
                labels_new.append(label_im[y,x])
    cv2.setMouseCallback('try', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    """
    calib_points=np.load("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/calib_coordinates.npy")
    for point in calib_points:
        for num, r in enumerate(regions):
            if point[0]>r.bbox[0] and point[0]<r.bbox[2] and point[1]>r.bbox[1] and point[1]<r.bbox[3]:
                masks_new.append(regions[num].convex_image)
                bbox_new.append(regions[num].bbox)   
                list_of_index_new.append(num)
                labels_new.append(label_im[point[1],point[0]])
    for item in labels_new:
        if labels_new.count(item)<2:
            labels_new.remove(item)
    list(dict.fromkeys(labels_new))
    rgb_mask_new = np.zeros_like(label_im)
    for x in labels_new:
        if x!=0:
            rgb_mask_new += (label_im==x).astype('uint8')
    rgb_mask_new = rgb_mask_new.astype('uint8')
    kernel = np.ones((11,11),np.uint8)
    rgb_mask_new=cv2.morphologyEx(rgb_mask_new, cv2.MORPH_CLOSE, kernel)
    red_new  =  imgor[:,:,0] * rgb_mask_new
    green_new = imgor[:,:,1] * rgb_mask_new
    blue_new  = imgor[:,:,2] * rgb_mask_new
    image_new = np.dstack([red_new, green_new, blue_new])
    cv2.imwrite("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/try.jpg",image_new)
    x=cv2.imread("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/try.jpg")
    ret,y = cv2.threshold(x,1,255,cv2.THRESH_BINARY)
    y=cv2.cvtColor(y, cv2.COLOR_BGR2GRAY)
    kernel2=np.ones((5,5),np.uint8)
    y = cv2.morphologyEx(y, cv2.MORPH_CLOSE, kernel2)
    imgfin=imgor.copy()
    for r in range(864):
        for c in range(648):
            if y[c,r]>=1:
                imgfin[c,r,1]=255
                imgfin[c,r,0]=255
                imgfin[c,r,2]=255
    cv2.imwrite("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/map.jpg",y)
    cv2.imwrite("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/generatedmap.jpg",imgfin)
    #cv2.waitKey(0)
class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(String, 'path', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
    	try:
            msg = String()
            img_path="/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/image.jpg"
            calibration=False
            dest=np.load("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/target_coordinates.npy")
            xfin=dest[0]
            yfin=dest[1]
            image=cv2.imread(img_path)
            image=cv2.resize(image,(864,648))
            type=cv2.aruco.DICT_5X5_50
            arucoDict = cv2.aruco.Dictionary_get(type)
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                parameters=arucoParams)
            if len(corners) > 0:
    # flatten the ArUco IDs list
                ids = ids.flatten()
    # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    cX = int((topLeft[0] + topRight[0]) / 2.0)
                    cY = int((topLeft[1] + topRight[1]) / 2.0)
                    cv2.rectangle(image,(min(topLeft[0],topRight[0],bottomRight[0],bottomLeft[0])-7,min(topLeft[1],topRight[1],bottomRight[1],bottomLeft[1])-7),(max(topLeft[0],topRight[0],bottomRight[0],bottomLeft[0])+7,max(topLeft[1],topRight[1],bottomRight[1],bottomLeft[1])+7),(255,255,255),-1)
                    cv2.imwrite("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/image2.jpg",image)
            xstart=cX #Take this point from hoverboard marker
            ystart=cY
            if abs(cX-xfin)>5 and abs(cY-yfin)>5:
                try:
                    shutil.rmtree("media")
                except:
                    print("Dir already clean")
                mapping("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/image2.jpg")
                RRT_call(xstart,ystart,xfin,yfin,"/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/map.jpg")
                path=np.load("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/found_path.npy")
                first_element=path[0]
                myradians = math.atan2(first_element[1]-ystart, first_element[0]-xstart)
                mydegrees = -math.degrees(myradians)
                msg.data = str(mydegrees)
                self.publisher_.publish(msg)
                self.get_logger().info('%s' % msg.data)
    	except:
            pass


def main(args=None):
    rclpy.init(args=args)

    path_publisher = PathPublisher()

    rclpy.spin(path_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
