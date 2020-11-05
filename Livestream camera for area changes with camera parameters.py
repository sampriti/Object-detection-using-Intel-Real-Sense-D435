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

# List to store parameter values
exposure=[0]
gain=[0]
laser=[0]
dsshift=[0]
dsneighbor=[0]
dssecondpeak=[0]
error_list=[0]


# Colorizer alignment which aligns depth map with color map
colorizer = rs.colorizer()

'''''''''''''''''''''''''''''''''''''''''''''''
Function to calculate the area for contour for each 
depth frame

count: Index of the frame
Return value:area in cm^2
'''''''''''''''''''''''''''''''''''''''''''''''

def depth_object(count,aligned_depth_frame,depth_frame):
    color = np.asanyarray(color_frame.get_data())
    depth=np.asanyarray((depth_frame).get_data())
    aligned_depth=np.asanyarray(aligned_depth_frame.get_data())
    FRAME_WIDTH = aligned_depth.shape[1]
    FRAME_HEIGHT = aligned_depth.shape[0]
    hfov=2*math.atan(FRAME_WIDTH/(2*fx))
    vfov=2*math.atan(FRAME_HEIGHT/(2*fx))


    # Colorizing the depth frame
    aligned_depth_colorized=np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
    plt.title('aligned depth hole frame')
    plt.imshow(aligned_depth_colorized)
    plt.show()

    # hole filling
    hole_filling = rs.hole_filling_filter()
    filled_depth = hole_filling.process((aligned_depth_frame))
    colorized_depth = np.asanyarray((filled_depth).get_data())
    # Overlaying the RGB image with the colorized image 
    dst1 = cv2.addWeighted(colorized_depth, 0.5, color, 0.5, 0.0)

    # Plot the Histogram of the Jet Colored map
    plt.title("hist")
    plt.hist(colorized_depth.ravel(),256,[0,256])
    plt.show()
    
    # Creating an empty image of size same as colorized depth image
    empty2 = np.zeros( colorized_depth.shape, dtype=np.uint8)


    # Canny edge dectection on an image   # Adjust the threshold according to the histogram 
    edges_empty = cv2.Canny(colorized_depth,200,300) # Canny edge image for some sigma
    

    #dilation
    kernel = np.ones((3, 3), np.uint8)  # define a kernel (block) to apply filters to
    dialated = cv2.dilate(edges_empty, kernel, iterations=2)
   

    # Finding contours 
    contours, _ = cv2.findContours(dialated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(contours, key=cv2.contourArea, reverse=True)  # get largest five contour area

    # Finding contour approx and hull from contours
    if cnts:
        epsilon = 0.01*cv2.arcLength(cnts[0],True)
        approx = cv2.approxPolyDP(cnts[1],epsilon,True)
        hull = cv2.convexHull(cnts[1])

        #  Drawing the approx on the empty image
        image = cv2.drawContours(empty2,[approx] , -1, (0, 255, 0), 2)
           
        
    # Finding moments of the required contour
    M=cv2.moments(hull)
    if M["m00"] != 0:
        cx=int(M['m10']/M['m00'])
        cy=int(M['m01']/M['m00'])
    else:
        # set values as what you need in the situation
        cx, cy = 0, 0
    center=(cx,cy)
    
    # Calculating distance of camera from center of image
    distance=  aligned_depth_frame.get_distance(cx,cy)*100  #in cm
    
    
    #  Finding contour area in pixels
    area=cv2.contourArea(approx)
    

    # Converting the area from pixels to cm^2
    HORIZONTAL_SCALING = 2 * math.tan(hfov / 2.0) / FRAME_WIDTH
    VERTICAL_SCALING = 2 * math.tan(vfov / 2.0) / FRAME_HEIGHT
    pix_width=HORIZONTAL_SCALING * distance
    pix_height = VERTICAL_SCALING * distance
    area_cm_2=area*pix_height*pix_width
    # print("area for depth is ",area)
    # print("area for depth in cm_2 is",area_cm_2)
    # print("pixel height in cm is",pix_height)
    # print("pixel width for cm is",pix_width)
    
    # Superimposing the image with contour over RGB image
    dst3 = cv2.addWeighted(empty2, 0.5, color, 0.5, 0.0)

    # Saving the above image with the frame number
    cv2.imwrite("depth_contour{}.jpg".format(count),empty2)
    
    # Displaying depth map,canny edge,dilated and final image with contour in one plot 
    f, axarr = plt.subplots(2,2)
    axarr[0,0].imshow(dst1)
    axarr[0,1].imshow(edges_empty)
    axarr[1,0].imshow(dialated)
    axarr[1,1].imshow(dst3)
    plt.show()
   
    # Returning area of object in cm^2 from depth map   
    
    return area_cm_2


index=0  # Frame index
laser_new=16

# Wait for first 5 frames to stabilise the frames
for x in range(5):
    pipeline.wait_for_frames()
    
# Control the camera parameters and find the error in area of object detected
while laser_new<250:
    frames = pipeline.wait_for_frames()
    align = rs.align(rs.stream.color)

    # align frameset of depth
    aligned_frameset = align.process(frames)


    # align depth frames,colorized depth map 
    aligned_depth_frame = aligned_frameset.get_depth_frame()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Extracting the intrinsics of camera
    intrin = color_frame.profile.as_video_stream_profile().intrinsics
    fx=intrin.fx   # focal length in x direction
    fy=intrin.fy   # focal length in y direction
    
    ppx=intrin.ppx 
    ppy=intrin.ppy
    
    # Keeping count of frames being observed
    index=index+1
       
    
    # Controlling the gain ( camera parameter)

    depth_sensor.set_option(rs.option.gain,laser_new)
    
    
    # Obtained the area of the current frame from the depth_object function
    depth_area=depth_object(index,aligned_depth_frame,depth_frame)
    

    # error in area ( area of object from depth map - actual area of the object)
    error=((depth_area-3.799)/(3.799))*100

    # Reading the previous value of Gain being recorded
    prev=gain[-1]
    
    # Reading the actual value of gain from the hardware
    curr=depth_sensor.get_option(rs.option.gain)
    

    # Recording the current parameter value in a list
    if prev!=curr:
        gain.append(curr)  #gain level
        # Recording error of current frame in the error list
        error_list.append(error)
    
    

# Increasing the value of gain parameter for next frame    
laser_new=laser_new+20

# Plotting the error in area against gain value
plt.plot(gain[1:], error_list[1:] )

# naming the x axis 
plt.xlabel('gain') 

# naming the y axis 
plt.ylabel('error area') 
plt.grid()

#saving the plot
plt.savefig('dssecondpeak_vs_error.png')
plt.show() 



# Cleanup:
pipeline.stop()
