import argparse
import pyrealsense2 as rs
import matplotlib.pyplot as plt  
import numpy as np
import cv2
import os
from time import time
import sys
import math
 
 # Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


# Start streaming
profile = pipeline.start(config)
device = profile.get_device()
depth_sensor = device.query_sensors()[0]
advnc_mode = rs.rs400_advanced_mode(device)

hole_list=[0]
exposure=[0]
gain=[0]
laser=[0]
dsshift=[0]
dsneighbor=[0]
dssecondpeak=[0]
error_list=[0]

colorizer = rs.colorizer()
lst_intensities=[]
'''''''''''''''''''''''''''''''''''''''''''''''
Function to calculate the area for contour for each 
depth frame

count: Index of the frame
Return value:area in cm^2
'''''''''''''''''''''''''''''''''''''''''''''''

def depth_object(count,aligned_depth_frame,depth_frame):
    # Converting the frames to numpy array for plotting using matplotlib
    color = np.asanyarray(color_frame.get_data())
    depth=np.asanyarray((depth_frame).get_data())
    aligned_depth=np.asanyarray(aligned_depth_frame.get_data())


    # Calculation of Horizontal and Vertical Field of View
    FRAME_WIDTH = aligned_depth.shape[1]
    FRAME_HEIGHT = aligned_depth.shape[0]
    hfov=2*math.atan(FRAME_WIDTH/(2*fx))
    vfov=2*math.atan(FRAME_HEIGHT/(2*fx))


    # Colorizing the aligned depth frame
    aligned_depth_colorized=np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
    plt.title('aligned depth hole frame')
    plt.imshow(aligned_depth_colorized)
    plt.show()
    


    # hole filling
    hole_filling = rs.hole_filling_filter()
    filled_depth = hole_filling.process((aligned_depth_frame))
    colorized_depth = np.asanyarray((filled_depth).get_data())



    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    im_color = cv2.applyColorMap(cv2.convertScaleAbs(colorized_depth, alpha=0.9), cv2.COLORMAP_JET)
    aligned_depth = (colorized_depth/256).astype('uint8')
    plt.title('Jet Colored map')
    plt.imshow(im_color)
    plt.show()
    

    # Plot the Histogram of the Jet Colored map
    plt.title("hist")
    plt.hist(im_color.ravel(),256,[0,256])
    plt.show()

    
    # Overlapping the RGB image with the colorized image
    dst1 = cv2.addWeighted(colorized_depth, 0.5, color, 0.5, 0.0)
    plt.title('blended')
    plt.imshow(dst1)
    plt.show()

    # Creating an empty image of size same as colorized depth image
    empty2 = np.zeros( aligned_depth.shape, dtype=np.uint8)


    # Canny edge dectection on an image
    edges_empty = cv2.Canny(im_color,230,250) # Canny edge image for some sigma
    plt.title('canny edge')
    plt.imshow(edges_empty)
    plt.show()
    

    # dilation
    kernel = np.ones((3, 3), np.uint8)  # define a kernel (block) for the filter
    #dialated = cv2.morphologyEx(edges_empty, cv2.MORPH_OPEN, kernel)
    dialated = cv2.dilate(edges_empty, kernel, iterations=2)
    plt.title('dialated')
    plt.imshow(dialated)
    plt.show()

    # Finding contours 
    contours, _ = cv2.findContours(dialated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # get largest five contour area
    cnts = sorted(contours, key=cv2.contourArea, reverse=True)  

    # Finding contour approx and hull from contours
    if cnts:
        epsilon = 0.01*cv2.arcLength(cnts[1],True)
        approx = cv2.approxPolyDP(cnts[1],epsilon,True)
        hull = cv2.convexHull(cnts[0])
        
        #  Drawing the approx on the empty image
        if cnts is not None:
            
            cv2.drawContours(empty2, [hull], -1, color=255, thickness=-1)

        # Access the image pixels inside contour boundary 
        pts = np.where(empty2 == 255)
      

        # Access the background pixels
        bk_pts=np.where(empty2 != 255)
      
        # Find distance of camera from background pixels
        for i in range(0,len(bk_pts[0])):
            bk_dist=  aligned_depth_frame.get_distance(bk_pts[1][i],bk_pts[0][i])*100  #in cm   #in cm
            
            distance_bk=bk_dist

        # Find distance of camera from pixels within contour
        dist_cap=[]
        for i in range(0,len(pts[0])):    
            dist_cap=aligned_depth_frame.get_distance(pts[1][(int)(len(pts[0])/2)],pts[0][(int)(len(pts[0])/2)])*100  #in cm 
            distance=dist_cap
        print("distance in cm for cap",distance)

               
        # Finding moments of the required contour
        M=cv2.moments(hull)
        # Finding the center of the contour
        if M["m00"] != 0:
            cx=int(M['m10']/M['m00'])
            cy=int(M['m01']/M['m00'])
        else:
            cx, cy = 0, 0
        center=(cx,cy)
        print("centre",cx," ",cy)
          
        # Converting the area from pixels to cm^2
        HORIZONTAL_SCALING = 2 * math.tan(hfov / 2.0) / FRAME_WIDTH
        VERTICAL_SCALING = 2 * math.tan(vfov / 2.0) / FRAME_HEIGHT
        
        
        # Superimposing the image with contour over RGB image
        dst3 = cv2.addWeighted(empty2, 0.5, color, 0.5, 0.0)
    
        # Saving the above image with the frame number
        cv2.imwrite("depth_contour{}.jpg".format(count),empty2)
        
       


        
        # Plotting the colored image,canny edge, contour on the same plot
        f, axarr = plt.subplots(2,2)
        axarr[0,0].imshow(im_color)
        axarr[0,1].imshow(edges_empty)
        axarr[1,0].imshow(empty2)
        
        plt.show()
       
    
    # Return the depth pf the cap being detected by contour method
    return distance_bk-distance




#try:
index=0  # Frame index
sum=0
cnt=0
DS_new=16
for x in range(50):
    pipeline.wait_for_frames()
    # print(x)
while laser_new<600:
    frames = pipeline.wait_for_frames()
    align = rs.align(rs.stream.color)
    # alig frameset of depth
    aligned_frameset = align.process(frames)


    # align depth frames,colorized depth map 
    aligned_depth_frame = aligned_frameset.get_depth_frame()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Calculating the intrinsics of camera
    intrin = color_frame.profile.as_video_stream_profile().intrinsics
    #print(intrin)
    fx=intrin.fx   # focal length in x direction
    fy=intrin.fy   # focal length in y direction
    fl=math.sqrt((fx*fx)+(fy*fy))
    ppx=intrin.ppx
    ppy=intrin.ppy
    
    
    index=index+1
    print("Frames Captured")
    
    # Controlling the camera parameter: DS second peak threshold for Intel RealSense D435i
   
    depth_control_control_group=advnc_mode.get_depth_control()
    depth_control_control_group.deepSeaSecondPeakThreshold=DS_new
    advnc_mode.set_depth_control(depth_control_control_group)
    

    # Depth of the object being viewed from the depth frame
    depth=depth_object(index,aligned_depth_frame,depth_frame)
    

    # error in Depth calculations  # 1.95cm is the ground truth depth of the object
    error=((depth-1.95)/(1.95))*100
 
    
    # Storing the previous value of the parameter
    prev=dssecondpeak[-1]
    
    # Reading the current value of DS second peak threshold
    curr=depth_control_control_group.deepSeaSecondPeakThreshold
    

    # Recording the current parameter value in a list
    if prev!=curr:
        dssecondpeak.append(curr)
    

    # Recording error of current frame in the error list
    error_list.append(error)
        
    
# Plot the Error in depth versus DS second peak
plt.plot(dssecondpeak[1:], error_list[1:] )

# naming the x axis 
plt.xlabel('dssecondpeak') 

# naming the y axis 
plt.ylabel('error area') 

plt.grid()

plt.savefig('dssecondpeak_vs_error.png')
plt.show() 

# sys.exit(1)

# Cleanup:
pipeline.stop()
