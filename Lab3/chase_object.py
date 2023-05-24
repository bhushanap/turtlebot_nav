# Lab 3, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# subscribe to object position from lidar
# control robot to chase object

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

import numpy as np

class ChaseObject(Node):
    
    def __init__(self):
        # Calls the superclass constructor from Node
        super().__init__('chase_object')
        # subscribes to /object/position topic
        self._position_subscriber = self.create_subscription(Point, 
                                                         '/object/lidar/position',
                                                         self._position_callback,
                                                         1)
        self._position_subscriber #to prevent unused variable warning
        
        # publishes Twist msg to /cmd_vel
        self._vel_publisher = self.create_publisher(Twist, 
                                                      '/cmd_vel',
                                                      5)
        self._vel_publisher # to prevent unused variable warning
        
        # Declares a velocity variable
        self.velocity = Twist()
        
        # Create a timer for publishing robot velocities
        timer_period = 1/1.25 # seconds = 1/f
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # declares a k-1 position variable
        # Point.x is i in pixels, Point.y is distance in meters, Point.z is theta in radians
        self.old_position = Point()
        self.cmd_distance = 0.5
        self.distance_envelope = 0.02
        
        self.cmd_theta = 0.0
        self.theta_envelope = 0.05
        
        # declare PID gains
        self.distance_kp = 1
        self.distance_ki = 0.03
        self.distance_kd = 0.01
        self.theta_kp = 2
        self.theta_ki = 0.01
        self.theta_kd = 0.01
        self.lin_limit = 0.2
        self.ang_limit = 2.3
        
        self.distance_integrator = 0.0
        self.theta_integrator = 0.0
        self.diff_timestep = 1.0/2.5 # seconds = 1/f
        
    def _position_callback(self, Point):
        self.get_logger().info('Received new object position at: i= %3.0f, d= %1.3f, theta= %1.3f' %(Point.x, Point.y, Point.z))
        self._cmd_vel(Point)
        #update 
        #self.object_local_position = Point
        
    def _cmd_vel(self, new_position):
        
        # If there is no object, stop 
        if np.isnan(new_position.y):
            self.velocity.linear.x = 0.0
            return
        elif np.isnan(new_position.z):
            self.velocity.angular.z = 0.0
            return
            
        distance_d = new_position.y
        error_distance = distance_d - self.cmd_distance
        self.distance_integrator += error_distance
        distance_diff = (distance_d - self.old_position.y) / self.diff_timestep
        print("distance error= ", error_distance)
        print("distance integrator= ", self.distance_integrator)
        print("distance differentiator= ", distance_diff)
        
        
        theta_d = new_position.z
        error_theta = theta_d - self.cmd_theta
        self.theta_integrator += error_theta
        theta_diff = (theta_d - self.old_position.z) / self.diff_timestep
        print("theta error= ", error_theta)
        print("theta integrator= ", self.theta_integrator)
        print("theta differentiator= ", theta_diff)

        # depreciate after implementation of object NaN
        if distance_d <= 0:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
        else:
            if self.velocity.linear.x>0:
                self.velocity.linear.x = min(self.lin_limit,self.distance_kp * error_distance + self.distance_ki*self.distance_integrator + self.distance_kd*distance_diff)
            else:
                self.velocity.linear.x = max(-self.lin_limit,self.distance_kp * error_distance + self.distance_ki*self.distance_integrator + self.distance_kd*distance_diff)
            if self.velocity.angular.z>0:
                self.velocity.angular.z = min(self.ang_limit,self.theta_kp * error_theta + self.theta_ki*self.theta_integrator + self.theta_kd*theta_diff)
            else:
                self.velocity.angular.z = max(-self.ang_limit,self.theta_kp * error_theta + self.theta_ki*self.theta_integrator + self.theta_kd*theta_diff)

        if abs(error_distance) < self.distance_envelope:
            self.distance_integrator = 0
        
        if abs(error_theta) < self.theta_envelope:
            self.theta_integrator = 0
            
        return            
        
    def timer_callback(self):
        self._vel_publisher.publish(self.velocity)
        self.get_logger().info('Published a velocity: linear x=%1.3f, angular z=%1.3f' %(self.velocity.linear.x, self.velocity.angular.z))
        
def main():
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    object_chaser = ChaseObject()
    
    # Trigger callback processing
    rclpy.spin(object_chaser)
    
            
    #clean up and shutdown
    object_chaser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
