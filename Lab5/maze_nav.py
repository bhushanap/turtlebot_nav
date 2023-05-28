# Lab 5, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Navigate Maze

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import numpy as np
import time

class NavigateMaze(Node):
    def __init__(self):
        # Calls the superclass constructor from Node
        super().__init__('NavigateMaze')
        
        self.goals = np.array([[1.75, 0.5],
                                     [4.0, 1.0],
                                     [4.5, -1.0],
                                     [0, 0]])
        self.goal_index = 0
        self.goal_point = self.goals[self.goal_index, :]
        # declare PoseStamped components
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = self.goal_point[0]
        self.goal_pose.pose.position.y = self.goal_point[1]
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0
        print('Initiated with goal (', self.goal_pose.pose.position.x, ', ', self.goal_pose.pose.position.y,')')
        
        # create subscription to goal reached topic
        self._goal_feedback_subscriber = self.create_subscription(NavigateToPose_FeedbackMessage, 
                                                                 '/navigate_to_pose/_action/feedback', 
                                                                 self._goal_callback, 1)
        self._goal_feedback_subscriber #to prevent unused variable warning
        # create publisher to goal pose channel
        self._goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        #self._goal_publisher.publish(self.goal_pose)
        #print('Published initial goal')
        
        # Create a timer for publishing goal
        goal_timer_period = 1/1.0 # seconds = 1/f
        self.goal_timer = self.create_timer(goal_timer_period, self.goal_timer_callback)
        
    def _goal_callback(self, NavigateToPose_FeedbackMessage):
        
        msg = NavigateToPose_FeedbackMessage
        
        if msg.feedback.distance_remaining < 0.5:
            self.goal_index += 1
            print('Updated with new goal', self.goals[self.goal_index, :])
            time.sleep(10)
        
        self.goal_point = self.goals[self.goal_index, :]
        # update PoseStamped components
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = self.goal_point[0]
        self.goal_pose.pose.position.y = self.goal_point[1]
        #print('Updated with new goal (', self.goal_pose.pose.position.x, ', ', self.goal_pose.pose.position.y,')')
        

        
    def goal_timer_callback(self):
        #print('Published a goal')
        self._goal_publisher.publish(self.goal_pose)

def main():
   # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    MazeNavigator = NavigateMaze()
    
    # Trigger callback processing
    rclpy.spin(MazeNavigator)
    
            
    #clean up and shutdown
    MazeNavigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
