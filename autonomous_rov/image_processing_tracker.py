#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image  # Update to sensor_msgs.msg.Image if using ROS2 standard image message
import numpy as np
import cv2
from . import camera_parameters as cam
from cv_bridge import CvBridge

def on_trackbar_change(x):
    pass

cv2.namedWindow('Result')

# Create trackbars for threshold values
cv2.createTrackbar('Hue_Lower', 'Result', 0, 179, on_trackbar_change)
cv2.createTrackbar('Hue_Upper', 'Result', 30, 179, on_trackbar_change)
cv2.createTrackbar('Saturation_Lower', 'Result', 100, 255, on_trackbar_change)
cv2.createTrackbar('Saturation_Upper', 'Result', 255, 255, on_trackbar_change)
cv2.createTrackbar('Value_Lower', 'Result', 100, 255, on_trackbar_change)
cv2.createTrackbar('Value_Upper', 'Result', 255, 255, on_trackbar_change)
# Hue: 170 (ranges from 0 to 179 in OpenCV)
# Saturation: 155 (ranges from 0 to 255)
# Value: 160 (ranges from 0 to 255)
set_desired_point = False #click right button to allow
get_hsv =False #click left button to allow
#desired_point = [300, 200]
mouseX, mouseY = 0, 0
hsv_value = [0, 0, 0]
# camera parameters
u0 = 320
v0 = 240
lx = 455
ly = 455
kud =0.00683 
kdu = -0.01424     
    
# convert a pixel coordinate to meters given linear calibration parameters
def convert2meter(pt,u0,v0,lx,ly):
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a pixel coordinate to meters using defaut calibration parameters
def convertOnePoint2meter(pt):
    global u0,v0,lx, ly
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a list of pixels coordinates to meters using defaut calibration parameters
def convertListPoint2meter (points):
    global u0,v0,lx, ly
    
    if(np.shape(points)[0] > 1):
        n = int(np.shape(points)[0]/2)
        point_reshaped = (np.array(points).reshape(n,2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convert2meter(pt,u0,v0,lx,ly)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter
#end of camera parameters 
def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4*scale+1), (b, g, r), -1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

def click_detect(event, x, y, flags, param):
    global get_hsv, set_desired_point, mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        get_hsv = True
        mouseX, mouseY = x, y
    if event == cv2.EVENT_RBUTTONDOWN:
        set_desired_point = True
        mouseX, mouseY = x, y

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        
        self.pub_tracked_point = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        self.pub_desired_point = self.create_publisher(Float64MultiArray, 'desired_point', 10)
        
        self.subscription = self.create_subscription(
            Image,                    # Message type
            'camera/image',  # Topic (assumed topic name)
            self.cameracallback,                # Callback function
            1                                   # Queue size (adjust if necessary)
        )
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format

        cv2.namedWindow("image")  # Create the window before setting the mouse callback
        cv2.setMouseCallback("image", click_detect)
        self.get_logger().info('Image processing node started')

    def cameracallback(self, msg):
        #self.get_logger().info('callback has started')

        global get_hsv, set_desired_point, desired_point, mouseX, mouseY, hsv_value

        image_np = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert ROS Image message to OpenCV image

        image_height, image_width, image_channels = image_np.shape
        desired_point= [image_width/2,image_height/2] #we are in a loop, next if-statement won't matter
        if set_desired_point:
            desired_point = [mouseX, mouseY]
            set_desired_point = False

        hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
        if get_hsv:
            hsv_value = hsv[mouseY, mouseX]  # Corrected order for indexing
            self.get_logger().info(f"HSV Value at ({mouseX}, {mouseY}): {hsv_value}")
            get_hsv = False

        #lower_bound = np.array([170, 155, 160])
        #upper_bound = np.array([179, 250, 255])
        tolerance = np.array([10, 50, 50])
        lower_bound = np.clip(hsv_value - tolerance, 0, 255)
        upper_bound = np.clip(hsv_value + tolerance, 0, 255)
        # Hue: 170 (ranges from 0 to 179 in OpenCV)
        # Saturation: 155 (ranges from 0 to 255)
        # Value: 160 (ranges from 0 to 255)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        cv2.imshow('Mask', mask)
        non_zero_pixels = cv2.findNonZero(mask)

        current_point = [0, 0]
        if non_zero_pixels is not None and len(non_zero_pixels) > 0:
            mean_point = np.mean(non_zero_pixels, axis=0, dtype=int)
            cx, cy = mean_point[0]
            cv2.circle(image_np, (cx, cy), 5, (0, 255, 0), -1)
            current_point = [cx, cy]
            overlay_points(image_np, current_point, 0, 255, 0, 'current tracked buoy')

        if current_point != [0, 0]:
            current_point_meter = convertOnePoint2meter(current_point)
            current_point_msg = Float64MultiArray(data=current_point_meter)
            self.pub_tracked_point.publish(current_point_msg)

        overlay_points(image_np, desired_point, 255, 0, 0, 'desired point')
        desired_point_meter = convertOnePoint2meter(desired_point)
        desired_point_msg = Float64MultiArray(data=desired_point_meter)
        self.pub_desired_point.publish(desired_point_msg)

        cv2.imshow("image", image_np)
        cv2.waitKey(2)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


