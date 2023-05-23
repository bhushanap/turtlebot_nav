# Lab 2, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# rotate robot to follow object

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class RotateRobot(Node):
    
    def __init__(self):
        # Calls the superclass constructor from Node
        super().__init__('rotate_robot')
        # subscribes to /object/position topic
        self._position_subscriber = self.create_subscription(Point, 
                                                         '/object/position',
                                                         self._position_callback,
                                                         1)
        self._position_subscriber
        # publishes to cmd/vel topic
        self._vel_publisher = self.create_publisher(Twist, 
                                                      '/cmd_vel',
                                                      5)
        self._vel_publisher # supposedly prevents unused variable warning
        
        # Declares an object_position variable
        self.object_position = Point()
        # Declares a velocity variable
        self.velocity = Twist()
        
        
    def _position_callback(self, Point):
        self.object_position = Point
        self.get_logger().info('Received an object at: x= %3.0f,y= %3.0f' %(self.object_position.x, self.object_position.y))
        self._cmd_vel()
    
    def _cmd_vel(self):
        if (self.object_position.x < 120.0):
            self.velocity.angular.z = 1.0
        elif (self.object_position.x > 200.0):
            self.velocity.angular.z = -1.0
        else:
            self.velocity.angular.z = 0
        self._vel_publisher.publish(self.velocity)
        self.get_logger().info('Published a velocity') 
        
    
def main():
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    robot_controller = RotateRobot()
    
    # Trigger callback processing
    rclpy.spin(robot_controller)
    
    # command robot to stop before exiting
    robot_controller.velocity.angular.z = 0
    robot_controller._vel_publisher.publish(robot_controller.velocity)
            
    #clean up and shutdown
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()