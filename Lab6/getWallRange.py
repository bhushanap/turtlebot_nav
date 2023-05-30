# Lab 4, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Publish wall distance given theta

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import numpy as np

class GetWallRange(Node):
    
    def __init__(self):
        print("Init from GetWallRange")
        # Calls the superclass constructor from Node
        super().__init__('getWallRange')
        
        #subscribe to lidar LaserScan msgs from /scan
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self._lidar_subscriber = self.create_subscription(LaserScan, 
                                            '/scan', 
                                            self._lidar_callback, 
                                            qos_profile)
        self._lidar_subscriber # to prevent unused variable warning
        
        # getWallRange node publishes a Point msg to the /wall/position topic
        self._wall_publisher = self.create_publisher(Point, 
                                                      '/wall/position',
                                                      1)
        self._wall_publisher # to prevent unused variable warning
        
        # Creates a timer for publishing wall position 
        timer_period = 1.0/2.5 # seconds = 1/f
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Create and intialize an wall position object
        #Point.x is empty, Point.y is distance in meters, Point.z is theta in radians
        self.wall_position = Point()
        self.wall_position.y = np.NaN
        self.wall_position.z = np.NaN
        

        
    def timer_callback(self):
        self._wall_publisher.publish(self.wall_position)
        #self.get_logger().info('Published object : d= %3.0f', theta= %3.0f %(self.wall_position.y, self.wall_position.z))


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
        
        # find index in radar and distance in radar, assign NaN for default condition of garbage data

        self.wall_position.y = np.NaN
        self.wall_position.z = np.NaN

        fov_ind = int(2.0 / LaserScan.angle_increment)

        #Lidar angle is clockwise
        #finding where the median wall is located with respect to lidar
        ranges = np.array(LaserScan.ranges)
        ranges = np.where(ranges==np.NaN, 10, ranges)

        ranges_fov = np.concatenate((ranges[:fov_ind], ranges[-fov_ind:]))
        
        index = (np.argpartition(ranges_fov, 5))[:5]
        index = index.astype(int)
        
        distance = float(np.median(ranges_fov[index]))
        ind = np.where(ranges==distance)[0][0]
        ang = ind * LaserScan.angle_increment
        
        # check if LaserScan got a valid return for camera reported theta
        if (LaserScan.range_min <= distance and distance <= LaserScan.range_max):
            self.wall_position.y = distance
            # convert clockwise 0 to 2pi lidar angles to clockwise -pi to pi angles
            if ang>np.pi:
                angle = -(2*np.pi - ang)
            else:
                angle = ang
            #if angle==0:
             #   angle = np.NaN
            self.wall_position.z = angle
             
        print("angle= ", self.wall_position.z)
        print("dist= ", self.wall_position.y)
        
        
    

def main():
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    wall_ranger = GetWallRange()
    
    # Trigger callback processing
    rclpy.spin(wall_ranger)
        
    #clean up and shutdown
    wall_ranger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
