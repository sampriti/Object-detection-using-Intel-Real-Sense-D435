import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import math
print("Environment Ready")

# Setup:
pipe = rs.pipeline()
cfg = rs.config()

# Read the bag file recorded from Intel RealSense Camera
cfg.enable_device_from_file("circle2.bag")
profile = pipe.start(cfg)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(5):
  pipe.wait_for_frames()
  
# Store next frameset for later processing:
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()
intrin = depth_frame.profile.as_video_stream_profile().intrinsics

fx=intrin.fx
fy=intrin.fy


# Cleanup:
pipe.stop()
print("Frames Captured")
depth=np.asanyarray(depth_frame.get_data())
color = np.asanyarray(color_frame.get_data())
# convert to RGB
image = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)



'''''''''''''''''''''''''''''''''''''''''''''''''''
Using Contour method to find the object in the image
'''''''''''''''''''''''''''''''''''''''''''''''''''

# convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

# create a binary thresholded image
_, binary = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)

plt.title('gray image')
plt.imshow(gray)
plt.show()



# find the contours from the thresholded image
contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#sorts contours according to their area from largest to smallest.
contor = sorted(contours, key = cv2.contourArea, reverse = True)[:10] 


# Finding the center of the contour
M=cv2.moments(contor[0])
if M["m00"] != 0:
    cx=int(M['m10']/M['m00'])
    cy=int(M['m01']/M['m00'])
else:
    # set values as what you need in the situation
    cx, cy = 0, 0

center=(cx,cy)

# Calculate the area of the contour
area=cv2.contourArea(contor[0])


#Calculate perimter of contour
perimeter = cv2.arcLength(contor[0],True)


# Wite perimter near the centre of the contour in the image
cv2.putText(image,str(perimeter),center2,cv2.FONT_HERSHEY_COMPLEX_SMALL,2,(255,0,0),3)

# draw all contours
image = cv2.drawContours(image, contor[1], -1, (0, 255, 0), 2)
cv2.imwrite("contour.jpg",image)


# show the image with contour and perimeter
plt.imshow(image)
plt.show()


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Hough Transform to find the circular object in the image for comparison with contour method
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

# detect circles in the image using hough lines techniques
# apply a blur using the median filter

img = cv2.medianBlur(gray, 5)

circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT, dp=0.9, minDist=80, param1=50, param2=30, maxRadius=0)


for co, i in enumerate(circles[0, :], start=1):

    # draw the outer circle in green

    cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)

    # draw the center of the circle in red

    cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)


# Save the image with hough circle    
cv2.imwrite("hough.jpg",image)
cv2.imshow("Image", image)

#plt.show()
cv2.waitKey(0)


