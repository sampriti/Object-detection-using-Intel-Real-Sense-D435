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

#  Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16,30)


# Start streaming
profile = pipeline.start(config)
device = profile.get_device()
depth_sensor = device.query_sensors()[0]
advnc_mode = rs.rs400_advanced_mode(device)


# List of camera parameters to analyse the hole fill ratio change with them
hole_list=[0]
exposure=[0]
gain=[0]
laser=[0]
dsshift=[0]
dsneighbor=[0]
dssecondpeak=[0]

def depth_object(count,aligned_depth_frame,depth_frame):
    # Get frameset of color and depth
    start = time()
    frames = pipeline.wait_for_frames()
    end = time()
    print(end-start)

    # Get aligned frames
    depth_frame = frames.get_depth_frame()
    # Converting to numpy array to render the image in opencv
    depth_image = np.asanyarray(depth_frame.get_data())

    # Create colorizer object
    colorizer = rs.colorizer()
    

    # create align object
    align = rs.align(rs.stream.color)

    # alig frameset of depth
    aligned_frameset = align.process(frames)

    # align depth frames,colorized depth map 
    aligned_depth_frame = aligned_frameset.get_depth_frame()
    
    # Converting to numpy array to render the image in opencv
    aligned_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
    # Plotting the aligned depth frame with holes
    plt.title('aligned depth hole frame')
    plt.imshow(aligned_depth)
    plt.show()
    

    # Calculate the no of holes in the image and the holes percent
    holes_number=0   # NO OF HOLES
    pixel_number=0   # NO OF VALID PIXELS
    for x in range(aligned_depth_frame.width):
        for y in range(aligned_depth_frame.height):
            depth = aligned_depth_frame.get_distance(x, y)
            if(depth==0):
                holes_number+=1
            else:
                pixel_number+=1    
    
    hole_percent=(holes_number/(holes_number+pixel_number))*100
    
    return hole_percent
        
#try:
i=1
sum=0
cnt=0
#Skip the first 5 frames for the noise to stabilise
for x in range(5):
    pipeline.wait_for_frames()
    
while i<10:
    frames = pipeline.wait_for_frames()
    align = rs.align(rs.stream.color)

    # align frameset of depth
    aligned_frameset = align.process(frames)
    # align depth frames,colorized depth map 
    aligned_depth_frame = aligned_frameset.get_depth_frame()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()


    # Read the intrisincs of camera from the depth frame
    intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    fx=intrin.fx
    fy=intrin.fy
    
    ppx=intrin.ppx
    ppy=intrin.ppy
    
    # keeping count of the frames being visualised
    i=i+1
    

    # Read the hole percentage from the depth object function defined above
    hole_percent=depth_object(i,aligned_depth_frame,depth_frame)
  
    # Collect the previous value of exposure( camera parameter)
    prev = exposure[-1]

    # Read the current value of exposure
    curr=aligned_depth_frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure)
    
    # Append the value of exposure and hole percentage only when there is change in exposure value
    if prev!=curr:
        exposure.append(aligned_depth_frame.get_frame_metadata(rs.frame_metadata_value.gain_level))
    
        hole_list.append(hole_percent)


# Plot the hole ratio versus exposure
plt.plot(hole_list[1:], exposure[1:] )

# # naming the x axis 
plt.xlabel('gain') 

# # naming the y axis 
plt.ylabel('hole percent') 

plt.savefig('gain_vs_hole.png')
plt.show() 



# Cleanup:
pipeline.stop()

