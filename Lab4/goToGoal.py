# Lab 4, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Go to goal while avoiding obstacle

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

import numpy as np
import time 

class GoToGoal(Node):
    
    def __init__(self):
        # Calls the superclass constructor from Node
        super().__init__('goToGoal')
        # subscribes to getObjectRange topic
        self._obstacle_subscriber = self.create_subscription(Point, 
                                                         '/obstacle/position',
                                                         self.update_obstacle,
                                                         1)
        self._obstacle_subscriber #to prevent unused variable warning
        # Declares an obstacle object
        self.obstacle = Point()
        
        self._odom_subscriber = self.create_subscription(Odometry, 
                                                         '/odom',
                                                         self.update_odometry,
                                                         1)
        self._odom_subscriber #to prevent unused variable warning
        
        # publishes Twist msg to /cmd_vel
        self._vel_publisher = self.create_publisher(Twist, 
                                                      '/cmd_vel',
                                                      5)
        self._vel_publisher # to prevent unused variable warning
        # Declares a velocity variable
        self.velocity = Twist()
        
        # Create a timer for publishing robot velocities
        vel_timer_period = 1/5.0 # seconds = 1/f
        self.vel_timer = self.create_timer(vel_timer_period, self.vel_timer_callback)
        
        # declare navigation variables
        # Initial odom position variables
        self.Init = True
        self.Init_ang = 0.0 
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_pos.z = 0.0
        # global position variables
        self.globalPos = Point()
        self.globalPos.x = 0.0
        self.globalPos.y = 0.0
        self.globalAng = 0.0
        
        #declare state machine
        self.state = "GTG"
        self.state_flag = False
        
        # declare goal management variables
        self.goal_array = [[1.55, 0.0], [1.5, 1.4], [0.0, 1.3], [0.0, 0.0]]
        #self.goal_array = []
        self.current_goal = Point()
        self.current_goal_index = 0
        self.goal_distance = 0.0
        self.goal_theta = 0.0
        self.goal_theta_local = 0.0
        
        # declare PID controller variables
        # Point.x is empty, Point.y is distance in meters, Point.z is theta in radians
        self.cmd_distance = 0.5
        self.cmd_theta = 0.0
        self.distance_envelope = 0.02
        self.theta_envelope = 0.1
        # k-1 position variable for Derivative Controller
        self.old_position = Point()
        # PID gains
        self.distance_kp = 0.5
        self.distance_ki = 0.03
        self.distance_kd = 0.01
        self.theta_kp = 3.0
        self.theta_ki = 0.01
        self.theta_kd = 0.01
        self.lin_limit = 0.15
        self.ang_limit = 0.35
        # PID integrator
        self.distance_integrator = 0.0
        self.theta_integrator = 0.0
        self.diff_timestep = 1.0/7.5 # seconds = 1/f

    def angle_wrap(self, angle):
        angle = angle % (2*np.pi)
        if angle > np.pi:
            angle -= 2*np.pi
        return angle
    
    def calc_distance(self, x_goal, y_goal, x_current, y_current):
        # calculates eucledian distance between two cartesian coordinates (x1, y1) and (x2, y2)
        return np.sqrt((x_goal-x_current)**2 + (y_goal-y_current)**2)
    
    def calc_theta(self, x_goal, y_goal, x_current, y_current):
        # calculates angle between two cartesian coordinates (x1, y1) and (x2, y2)
        return (np.arctan2((y_goal - y_current),(x_goal - x_current)))


    def exit(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        return
        
    def avoid_obstacle_state(self):
        # Pure Obstacle Avoidance behavior
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        #if obstacle to the left, turns towards it until its within .1rad or 5 degrees of nose of robot
        if self.obstacle.z>0.1:
            self.velocity.angular.z = min(self.ang_limit,self.theta_kp*self.obstacle.z)
        #if obstacle to the right, turns towards it until its within .1rad or 5 degrees of nose of robot
        elif self.obstacle.z<-0.1:
            self.velocity.angular.z = max(-self.ang_limit,self.theta_kp*self.obstacle.z)
        # if obstacle straight ahead, back up
        else:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = -0.06
        #back up until 0.35m from obstacle, then go to OF state
        if self.obstacle.y > 0.35:
                self.state="OF"
    
   
    
    def follow_wall_CW_state(self):
        self.velocity.linear.x = 0.00
        self.velocity.angular.z = 0.00
        
        # If 0.45m far from obstacle, stop following wall
        if self.obstacle.y > 0.45:
            self.state="STOP"
        #If closer to goal than obstacle, stop following obstacle
        elif self.goal_distance < self.obstacle.y:
            self.state="STOP"
          
        #calculate a CW heading angle orthogonal to obstacle angle (which is negative)
        heading_angle = self.angle_wrap(self.obstacle.z + np.pi/2)
        print("heading angle = ",heading_angle, " = ", self.obstacle.z, " + ", np.pi/2)
                  
        #as we turn left, obstacle angle grows towards -pi/2 and heading angle decrease towards 0
        if heading_angle > 0:
            self.velocity.angular.z = min(self.ang_limit,self.theta_kp * heading_angle)
        #don't think this condition exists for positive heading angle    
        elif heading_angle < 0:
            self.velocity.angular.z = max(-self.ang_limit,self.theta_kp * heading_angle)
        # when perpendicular to obstacle, drive forward
        # elif heading_angle >= -self.theta_envelope and heading_angle <= self.theta_envelope:
        #     self.velocity.angular.z = 0.0
        #     self.velocity.linear.x = 0.2

        if abs(heading_angle) < 0.15:
            self.velocity.linear.x = 0.15
        


    def follow_wall_CCW_state(self):
        self.velocity.linear.x = 0.00
        self.velocity.angular.z = 0.00
        
        # If 0.45m far from obstacle, stop following wall
        if self.obstacle.y > 0.45:
            self.state="STOP"
        #If closer to goal than obstacle, stop following obstacle
        elif self.goal_distance < self.obstacle.y:
            self.state="STOP"
            
        #calculate a CCW heading angle orthogonal to obstacle angle (which is positive)
        heading_angle = self.angle_wrap(self.obstacle.z - np.pi/2)
        print("heading angle = ",heading_angle, " = ", self.obstacle.z, " - ", np.pi/2)
                    
        #as we turn right, obstacle angle grows towards pi/2 and heading angle increase towards 0
        if heading_angle < 0:
            self.velocity.angular.z = max(-self.ang_limit,self.theta_kp * heading_angle)
        #don't think this condition exists for negative heading angle    
        elif heading_angle > 0:
            self.velocity.angular.z = min(self.ang_limit,self.theta_kp * heading_angle)   
        # when perpendicular to obstacle, drive forward              
        # elif heading_angle >= -self.theta_envelope and heading_angle <= self.theta_envelope :
        #     self.velocity.angular.z = 0.0
        #     self.velocity.linear.x = 0.2

        if abs(heading_angle) < 0.15:
            self.velocity.linear.x = 0.15
 
    
    def go_to_goal_state(self):
        # Pure Go To Goal behavior  
        # If Initializing, load goal waypoints
        if self.Init:
            print("current goal array ", self.goal_array)
            self.current_goal_index = 0
            print("current goal ", self.current_goal_index)   
        # while we are still in GTG state and have not reached current goal
        # calls PID controller to keep driving towards current goal
        if (self.goal_distance < self.distance_envelope):
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0 
            self.current_goal_index += 1
            print("At global coords x= ", self.globalPos.x, ", y= ", self.globalPos.y)
            print("Got to goal, new goal is ", self.current_goal_index)
            print("transitioning to STOP state")
            self.state = "STOP"
            self.state_flag = True
            return

        elif  self.goal_distance >=5.0:
            print("Lost, going back to first goal")
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.current_goal_index = 0
            
        elif (self.goal_distance >= self.distance_envelope):
            print("PID to goal distance = ", self.goal_distance)
            self.velocity.linear.x = min(self.lin_limit,self.distance_kp * self.goal_distance)
            if self.goal_theta_local>0.0:
                self.velocity.angular.z = min(self.ang_limit,self.theta_kp * self.goal_theta_local)
            else:
                self.velocity.angular.z = max(-self.ang_limit,self.theta_kp * self.goal_theta_local)
        return
        

    def obstacle_found_state(self):
        # State to react to obstacles
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        #print("updated obstacle distance inside OF = ", self.obstacle.y)
        #print("updated obstacle angle inside OF = ", self.obstacle.z)
        
        # check if obstacle is still in front of robot
        if self.obstacle.y<0.4:
            #if obstacle still in the way, choose left or right based on direction of goal
            if abs(self.obstacle.z) < (np.pi/2 + 2*self.theta_envelope):
                self.dir = self.goal_theta_local-self.obstacle.z
                print("dir = ",self.dir)
                #goal is to the left of obstacle
                if self.dir>0:
                    self.state="FWCW"
               #goal is to the right of obstacle         
                elif self.dir<=0:
                    self.state="FWCCW"
            else:
                self.state="STOP"
        else:
            self.state="STOP"

    def stop_state(self):
        # stop, update goal, and face next goal
        
        #stop
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        # Check if facing next goal
        if abs(self.goal_theta_local) <= self.theta_envelope:
            print("facing next goal")
            self.velocity.angular.z = 0.0
            self.state = "GTG"
        #turn towards next goal
        else:
            if self.goal_theta_local>0:
                self.velocity.angular.z = min(self.ang_limit,self.theta_kp * self.goal_theta_local)
            else:
                self.velocity.angular.z = max(-self.ang_limit,self.theta_kp * self.goal_theta_local)
        
    
    def update(self):
        #Updates navigation variables
        
        # If trying to transition to a 4th waypoint, course complete DONE
        if (self.current_goal_index) == 3:
            self.state="DONE"
        #update current goal coordinates    
        self.current_goal.x = self.goal_array[self.current_goal_index][0]
        self.current_goal.y = self.goal_array[self.current_goal_index][1]
        #update relative distances and angles of robot and goals in local and global frames
        self.goal_distance = self.calc_distance(self.current_goal.x, self.current_goal.y, self.globalPos.x, self.globalPos.y)
        self.goal_theta = self.calc_theta(self.current_goal.x, self.current_goal.y, self.globalPos.x, self.globalPos.y)
        self.goal_theta_local = self.goal_theta - self.globalAng
        if abs(self.goal_theta_local)>np.pi:
            self.goal_theta_local = ((2*np.pi)-abs(self.goal_theta_local))*(-self.goal_theta_local/abs(self.goal_theta_local))
        
        if self.state_flag == True:
            time.sleep(10)
            self.state_flag = False

        print("current goal ", self.current_goal_index)
        print("State Machine in state ", self.state)
        print("goal distance ", self.goal_distance)
        print("goal theta ", self.goal_theta)
        print("global Angle ", self.globalAng)
        print("goal theta local frame ", self.goal_theta_local)
    

    def update_odometry(self, Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        # wrap global angle from 0 to 2 pi to -pi to pi
        if self.globalAng%(2*np.pi) > np.pi:
            self.globalAng = (self.globalAng%(2*np.pi) - 2*np.pi)
        else:
            self.globalAng = self.globalAng%(2*np.pi)
        
    def update_obstacle(self, Point):
        # update obstacle variable from lidar callback
        self.obstacle = Point
        #print("updated obstacle distance = ", self.obstacle.y)
        #print("updated obstacle angle = ", self.obstacle.z)
        
        # found an obstacle in front sector of robot
        if abs(self.obstacle.z) < (np.pi/2 + self.theta_envelope):

            if self.obstacle.y < 0.40 and self.state!="FWCW" and self.state!="FWCCW":
                self.state="OF"
            else:
                return
        return   

    def vel_timer_callback(self):
                       
        self._vel_publisher.publish(self.velocity)
        self.get_logger().info('Published a velocity: linear x=%1.3f, angular z=%1.3f' %(self.velocity.linear.x, self.velocity.angular.z))
        self.update()

        # State Machine
        if self.state=="GTG":
            self.go_to_goal_state()
        elif self.state=="AO":
            self.avoid_obstacle_state()
        elif self.state=="FWCW":
            self.follow_wall_CW_state()
        elif self.state=="FWCCW":
            self.follow_wall_CCW_state()
        elif self.state=="STOP":
            self.stop_state()
        elif self.state=="OF":
            self.obstacle_found_state()
        elif self.state=="DONE":
            self.exit()    

 

def main():
    # init routine needed for ROS2
    rclpy.init() 
    # Create a class object to be used
    GoalChaser = GoToGoal()
    
    # Trigger callback processing
    rclpy.spin(GoalChaser)
    
            
    #clean up and shutdown
    GoalChaser.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()