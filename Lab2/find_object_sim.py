# Lab 2, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Subscribe to camera
# Find ball
# Publish ball coordinates

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import numpy as np
import cv2
from cv_bridge import CvBridge

class FindObject(Node):
    
    def __init__(self):
        # Calls the superclass constructor from Node
        super().__init__('find_object')
        #print('init from find_object')
        
        # Declare that the find_object node is subscribing to the /camera/image_raw topic
        self._img_subscriber = self.create_subscription(Image, 
                                                         '/camera/image_raw',
                                                         self._image_callback,
                                                         1)
        self._img_subscriber # supposedly prevents unused variable warning
        
        # Declare that the find_object node is publishing to the /object/position topic
        self._point_publisher = self.create_publisher(Point, 
                                                      '/object/position',
                                                      1)
        self._point_publisher # supposedly prevents unused variable warning
        
        # Create a timer for publishing object position
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Create and intialize a current object position
        self.object_position = Point()
        self.object_position.x = 0.0
        self.object_position.y = 0.0
        self.object_position.z = 0.0
        
        
    def _image_callback(self, Image):
        #print('image_callback from find_object')
        self._imgBGR = CvBridge().imgmsg_to_cv2(Image, "bgr8")
        self.object_position = self.find_ball(self._imgBGR)
             
    def find_ball(self, imgBGR):
        #print('find_ball from find_object')
        #declaring a local variable coord
        coord = Point()
        
        # to grayscale
        gray = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2GRAY)
       
        # make a 5x5 kernel
        kernel = np.ones((5, 5), np.uint8)
    
        # blur the image
        blur = cv2.blur(gray,(5,5))
    
        # take a binary threshold
        ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)
    
        #Opening and closing
        dilate1 = cv2.dilate(thresh, kernel, iterations=1)
        erode1 = cv2.erode(dilate1, kernel, iterations=2)
        dilate2 = cv2.dilate(erode1, kernel, iterations=1)
    
        #optimised parameters for hough circles
        circles = cv2.HoughCircles(dilate2, cv2.HOUGH_GRADIENT, dp=2, minDist=480, param1=100, param2=50, minRadius=50, maxRadius=200)      
        try:           
            coord.x = float(circles[0, 0])
            coord.y = float(circles[0, 1])
        except TypeError:
            #self.get_logger().info('No object found')
            return Point(x=220.0, y=100.0)
        else:
            self.get_logger().info('Found an object at: x= %3.0f,y= %3.0f' %(self.object_position.x, self.object_position.y))
            return coord
    
    def timer_callback(self):
        self._point_publisher.publish(self.object_position)
        self.get_logger().info('Published object position: x= %3.0f,y= %3.0f' %(self.object_position.x, self.object_position.y))
        
               
     
        
def main():
    #print('main from team69_object_follower.')
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    object_finder = FindObject()
    
    # Trigger callback processing
    rclpy.spin(object_finder)
        
    #clean up and shutdown
    object_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
