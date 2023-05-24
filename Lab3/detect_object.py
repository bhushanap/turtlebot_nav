# Lab 3, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Subscribe to camera
# Find object
# Publish object x

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

import numpy as np

import cv2
from cv_bridge import CvBridge

class DetectObject(Node):
    
    def __init__(self):
        #print('init from detect_object')
        # Calls the superclass constructor from Node
        super().__init__('detect_object')
        
        # detect_object node subscribes to the /camera/image/compressed topic for a CompressedImage msg and calls _image_callback 
        self._img_subscriber = self.create_subscription(CompressedImage, 
                                                         '/camera/image/compressed',
                                                         self._image_callback,
                                                         1)
        self._img_subscriber # to prevent unused variable warning
        
        # detect_object node publishes a Point msg to the /object/camera/position topic
        self._point_publisher = self.create_publisher(Point, 
                                                      '/object/camera/position',
                                                      1)
        self._point_publisher # to prevent unused variable warning
        
        # detect_object node publishes a CompressedImage msg to the /object/image/compressed topic
        self._img_publisher = self.create_publisher(CompressedImage, 
                                                    '/object/image/compressed',
                                                    1)
        self._img_publisher # to prevent unused variable warning
        
        # Creates a timer for publishing object position and object image
        # Camera is configured to 30 fps
        timer_period = 1.0/15 # seconds = 1/f
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Create and intialize an object pixel position in middle of frame
        # Point.x is i in pixels, Point.y is j in pixels
        self.object_pixel_position = Point()
        self.object_pixel_position.x = 160.0
        self.object_pixel_position.y = 120.0
        
        # Create and initialize an object local position
        #Point.x is i in pixels, Point.y is distance in meters, Point.z is theta in radians
        self.object_local_position = Point()
        self.object_local_position.x = 160.0
        self.object_local_position.y = 0.0
        self.object_local_position.z = (np.pi)*0
        
        # initialize a processed frame image for debugging
        self.processed_frame = CompressedImage()
        
                      
        # processes incoming images on camera/image/compressed topic
    def _image_callback(self, CompressedImage):
        #print('image_callback from detect_object')
        # converts incoming subscribed ROS image to CV2 image
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        # initializes processed frame as CV2 version of incoming ROS image
        self.processed_frame = CvBridge().cv2_to_compressed_imgmsg(self._imgBGR)
        # calls ball detection algorithm
        self.object_pixel_position = self.find_ball(self._imgBGR)
        # updates object local position
        self.object_local_position.x = self.object_pixel_position.x
        self.object_local_position.z = self.get_theta(self.object_pixel_position.x)
        
        
    def find_ball(self, imgBGR):
        #print('find_ball from detect_object')
        #declares a coordinate variable
        pixel_coord = Point()
        
        # converts bgr8 image to grayscale
        gray = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2GRAY)
       
        # makes a 5x5 kernel
        kernel = np.ones((5, 5), np.uint8)
    
        # blurs the image
        blur = cv2.blur(gray,(5,5))
    
        # takes a binary threshold
        ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)
    
        # performs image Opening and closing
        dilate1 = cv2.dilate(thresh, kernel, iterations=1)
        erode1 = cv2.erode(dilate1, kernel, iterations=2)
        dilate2 = cv2.dilate(erode1, kernel, iterations=1)
        # output of image processing pre detection algorithm
        final_img = dilate2
        
        # calls hough circles algorithm with optimised parameters for camera resolution and ball size
        circles = cv2.HoughCircles(final_img, cv2.HOUGH_GRADIENT, dp=2, minDist=250, param1=100, param2=50, minRadius=25, maxRadius=100)       
        try:
            # rounds circles array of 3 float vectors to array of 3 int vectors, then converts them to array of unsigned 16 bit integer vector
            circles = np.uint16(np.around(circles))
            # assign the last circle (if multiple) x, y to coord and draws on processed frame
            for i in circles[0, :]:                
                pixel_coord.x = float(i[0])
                pixel_coord.y = float(i[1])
                #draw the circle  
                cv2.circle(final_img,(i[0], i[1]),i[2],(0,255,0),2)
                #draw circle center
                cv2.circle(final_img,(i[0], i[1]),2,(0,0,255),2)
                # Make processed frame a ROS image
                self.processed_frame = CvBridge().cv2_to_compressed_imgmsg(final_img)
                
        except TypeError as e:
            self.get_logger().info('No object found')
            #pixel_coord.x = 160.0
            #pixel_coord.y = 120.0
            pixel_coord.x = np.NaN
            return pixel_coord
        else:
            #self.get_logger().info('Found an object at: x= %3.0f,y= %3.0f' %(pixel_coord.x, pixel_coord.y))
            return pixel_coord
    
    def timer_callback(self):
        self._point_publisher.publish(self.object_local_position)
        self.get_logger().info('Published object : i= %3.0f, d= %3.0f, theta= %1.3f' %(self.object_local_position.x, self.object_local_position.y, self.object_local_position.z))
        self._img_publisher.publish(self.processed_frame)
        #self.get_logger().info('Published processed frame')
               
    def get_theta(self, x):
        # assume linear lens and 62.2 degrees Horizontal Field of view and 48.8 degrees vertical Field of view
        #for (320, 240) (i, j) raspicam resolution
        theta_degrees = -(x - 160) * (62.2/320)
        print("theta in degrees= ", theta_degrees)
        return np.deg2rad(theta_degrees)
        
def main():
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    object_detector = DetectObject()
    
    # Trigger callback processing
    rclpy.spin(object_detector)
        
    #clean up and shutdown
    object_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
