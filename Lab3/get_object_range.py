# Lab 3, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Subscribe to object position from camera
# Find object distance for angular position from lidar
# Publish object i, distance, theta

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import numpy as np

class RangeObject(Node):
    
    def __init__(self):
        #print('init from get_object_range')
        # Calls the superclass constructor from Node
        super().__init__('get_object_range')
        
        #subscribe to Lidar LaserScan msgs from /scan
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self._lidar_subscriber = self.create_subscription(LaserScan, 
                                            "/scan", 
                                            self._lidar_callback, 
                                            qos_profile)
        self._lidar_subscriber # to prevent unused variable warning
        
        # subscribes to Point msgs from /object/camera/position topic
        self._camera_position_subscriber = self.create_subscription(Point, 
                                                         '/object/camera/position',
                                                         self._camera_position_callback,
                                                         1)
        self._camera_position_subscriber
        
        # get_object_range node publishes a Point msg to the /object/lidar/position topic
        self._point_publisher = self.create_publisher(Point, 
                                                      '/object/lidar/position',
                                                      1)
        self._point_publisher # to prevent unused variable warning
        
        # Creates a timer for publishing object lidar position 
        timer_period = 1.0/2.5 # seconds = 1/f
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Create and intialize an object local position object
        #Point.x is i in pixels, Point.y is distance in meters, Point.z is theta in radians
        self.object_local_position = Point()
        

        
    def timer_callback(self):
        self._point_publisher.publish(self.object_local_position)
        #self.get_logger().info('Published object : i= %3.0f, d= %3.0f', theta= %3.0f %(self.object_local_position.x, self.object_local_position.y, self.object_local_position.z))

    def _lidar_callback(self, LaserScan):
        #Laserscan msg format:
        #float32 angle_min
        #float32 angle_max
        #float32 angle_increment
        #float32 time_increment
        #float32 scan_time
        #float32 range_min
        #float32 range_max
        #float32[] ranges
        #float32[] intensities
     
        # find index in radar returns corresponding to camera reported theta
        if np.isnan(self.object_local_position.z):
            self.object_local_position.y = np.NaN
            return
        
        normalized_angle = self.object_local_position.z % (2*np.pi)
        mid_index =  normalized_angle / LaserScan.angle_increment
        min_index = int(np.floor(mid_index))
        max_index = int(np.ceil(mid_index)) + 1
        #print("mid_index= %3.0f, min_index= %3.0f, max_index= %3.0f" %(mid_index, min_index, max_index))
        
        # find median distance of values at that index
        distance = float(np.median(LaserScan.ranges[min_index : max_index]))
        print("object distance= ", distance)
        # check if LaserScan got a valid return for camera reported theta
        if (LaserScan.range_min <= distance and distance <= LaserScan.range_max):
            self.object_local_position.y = distance
            return 
        else:
            self.object_local_position.y = 0.0
            return
    
    def _camera_position_callback(self, Point):
        self.object_local_position.x = Point.x
        self.object_local_position.z = Point.z
        
        
    

def main():
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    object_ranger = RangeObject()
    
    # Trigger callback processing
    rclpy.spin(object_ranger)
        
    #clean up and shutdown
    object_ranger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
