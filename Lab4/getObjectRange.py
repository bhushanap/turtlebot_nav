# Lab 4, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Publish obstacle distance given theta

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import numpy as np

class GetObjectRange(Node):
    
    def __init__(self):
        print("Init from GetObjectRange")
        # Calls the superclass constructor from Node
        super().__init__('getObjectRange')
        
        #subscribe to lidar LaserScan msgs from /scan
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self._lidar_subscriber = self.create_subscription(LaserScan, 
                                            '/scan', 
                                            self._lidar_callback, 
                                            qos_profile)
        self._lidar_subscriber # to prevent unused variable warning
        
        # getObjectRange node publishes a Point msg to the /obstacle/position topic
        self._obstacle_publisher = self.create_publisher(Point, 
                                                      '/obstacle/position',
                                                      1)
        self._obstacle_publisher # to prevent unused variable warning
        
        # Creates a timer for publishing obstacle position 
        timer_period = 1.0/2.5 # seconds = 1/f
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Create and intialize an obstacle position object
        #Point.x is empty, Point.y is distance in meters, Point.z is theta in radians
        self.obstacle_position = Point()
        self.obstacle_position.y = np.NaN
        self.obstacle_position.z = np.NaN
        

        
    def timer_callback(self):
        self._obstacle_publisher.publish(self.obstacle_position)
        #self.get_logger().info('Published object : d= %3.0f', theta= %3.0f %(self.obstacle_position.y, self.obstacle_position.z))


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

        self.obstacle_position.y = np.NaN
        self.obstacle_position.z = np.NaN

        fov_ind = int(2.0 / LaserScan.angle_increment)

        #Lidar angle is clockwise
        #finding where the median obstacle is located with respect to lidar
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
            self.obstacle_position.y = distance
            # convert clockwise 0 to 2pi lidar angles to clockwise -pi to pi angles
            if ang>np.pi:
                angle = -(2*np.pi - ang)
            else:
                angle = ang
            #if angle==0:
             #   angle = np.NaN
            self.obstacle_position.z = angle
             
        print("angle= ", self.obstacle_position.z)
        print("dist= ", self.obstacle_position.y)
        
        
    

def main():
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    obstacle_ranger = GetObjectRange()
    
    # Trigger callback processing
    rclpy.spin(obstacle_ranger)
        
    #clean up and shutdown
    obstacle_ranger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
