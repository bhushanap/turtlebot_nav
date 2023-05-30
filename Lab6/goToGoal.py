# Final, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Follow signs to navigate maze

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String

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

        # subscribes to getWallRange topic
        self._wall_subscriber = self.create_subscription(Point, 
                                                         '/wall/position',
                                                         self.update_wall,
                                                         1)
        self._wall_subscriber #to prevent unused variable warning
        # Declares a wall object
        self.wall = Point()
        
        self._odom_subscriber = self.create_subscription(Odometry, 
                                                         '/odom',
                                                         self.update_odometry,
                                                         1)
        self._odom_subscriber #to prevent unused variable warning
                                                                
        self._sign_subscriber = self.create_subscription(String,   
                                                         '/sign/label',
                                                         self.update_sign,
                                                         1)
        #self._sign_subscriber #to prevent unused variable warning

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
        self.prevAng = self.globalAng
        #declare state machine
        self.state = "GTG"
        self.sign = None
        self.signlist = []
        
        # declare PID controller variables
        # Point.x is empty, Point.y is distance in meters, Point.z is theta in radians
        # self.cmd_distance = 0.5
        # self.cmd_theta = 0.0
        self.distance_envelope = 0.02
        self.theta_envelope = 0.005
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
        # counting how many times in LOST state loop
        self.times_lost = 0

    def angle_wrap(self, angle):
        angle = angle % (2*np.pi)
        if angle > np.pi:
            angle -= 2*np.pi
        return angle
    

    def exit(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        return
        
    

    def find_wall(self):
        # Stop initially if not stopped
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        #if wall to the left, turns towards it until its within .1rad or 5 degrees of nose of robot
        if self.wall.z>10*self.theta_envelope:
            self.velocity.angular.z = min(self.ang_limit,self.theta_kp*self.wall.z)
        #if wall to the right, turns towards it until its within .1rad or 5 degrees of nose of robot
        elif self.wall.z<-10*self.theta_envelope:
            self.velocity.angular.z = max(-self.ang_limit,self.theta_kp*self.wall.z)
        # if wall straight ahead, back up
        else:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = -0.06
        #back up until 0.44m from wall, then go to KNN state
        if self.wall.y > 0.44:
            self.times_lost += 1
            self.state="KNN"
        

    def go_straight(self):
        # Go Straight till a wall is found 
        error = self.obstacle.y - 0.43
        self.velocity.linear.x = min(error*self.distance_kp,self.lin_limit)
        self.velocity.angular.z = 0.0
        
        #keep correcting angle
        self.goalAng = self.prevAng
        error = self.angle_wrap(self.goalAng-self.globalAng)
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp,self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp,-self.ang_limit)
        else:
            #turn complete now go straight
            self.velocity.angular.z = 0.0
   
        # found an obstacle in front of robot
        if abs(self.obstacle.y) < 0.45:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.state="KNN"
            return 

    def knn(self):
        # stop, detect sign, and move to rotate state
        self.confident = False
        #stop
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        # append detected sign here
        self.signlist.append(self.sign)
        #self.signlist = ["left","left","left","left"]
        self.prevAng = self.globalAng
        #when a sign is found correctly thrice, robot is confident
        if len(self.signlist) >= 3:
            if self.signlist[-1] == self.signlist[-2] and self.signlist[-2] == self.signlist[-3]:
                self.confident = True
                sign = self.signlist[-1]
                self.signlist = []
        # sign is right, left or reverse: accordingly update state
        if self.confident == True:
            if sign == "right":
                self.state = "RTURN"
            elif sign == "left":
                self.state = "LTURN"
            elif self.times_lost >= 2:
                self.state = "LTURN"
            elif sign == "reverse":
                self.state = "UTURN"
            elif sign == "stop":
                self.state = "DONE"
            else:
                self.state = "LOST"
        if len(self.signlist) >= 10:
            self.state = "LOST"
        
    def lturn(self):
        # reset LOST state counter
        self.times_lost = 0
        #define goal angle 90deg to left and turn
        self.goalAng = self.prevAng + np.pi/2
        error = self.angle_wrap(self.goalAng-self.globalAng)
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp,self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp,-self.ang_limit)
        else:
            #turn complete now go straight
            self.velocity.angular.z = 0.0
            self.state = "GTG"
            self.prevAng = self.globalAng

    def rturn(self):
        # reset LOST state counter
        self.times_lost = 0
        #define goal angle 90deg to right and turn
        self.goalAng = self.prevAng - np.pi/2
        error = self.angle_wrap(self.goalAng-self.globalAng)
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp,self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp,-self.ang_limit)
        else:
            #turn complete now go straight
            self.velocity.angular.z = 0.0
            self.state = "GTG"
            self.prevAng = self.globalAng

    def uturn(self):
        # reset LOST state counter
        self.times_lost = 0
        #define goal angle 180deg to left and turn
        self.goalAng = self.prevAng + np.pi
        error = self.angle_wrap(self.goalAng-self.globalAng)
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp,self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp,-self.ang_limit)
        else:
            #turn complete now go straight
            self.velocity.angular.z = 0.0
            self.state = "GTG"
            self.prevAng = self.globalAng

    def update(self):
        #Print navigation variables in logger

        print("State Machine in state ", self.state)
        print("front wall distance ", self.obstacle.y)
        print("global Angle ", self.globalAng)
        print("sign ", self.sign)
        print("signlist ", self.signlist)
        print("nearest wall angle ", self.wall.z)
        print("nearest wall distance ", self.wall.y)
        #conditions to check for lost state, weird sensor values? nan, None check conditions...
        # if true
        # self.state = "LOST"


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

        self.globalAng = self.angle_wrap(self.globalAng)
        
    def update_obstacle(self, Point):
        # update obstacle variable from lidar callback
        self.obstacle = Point

    def update_wall(self, Point):
        # update wall variable from lidar callback
        self.wall = Point
          
    def update_sign(self, Sign):
        #update sign value from camera node
        self.sign = Sign.data

    def vel_timer_callback(self):
                       
        self._vel_publisher.publish(self.velocity)
        self.get_logger().info('Published a velocity: linear x=%1.3f, angular z=%1.3f' %(self.velocity.linear.x, self.velocity.angular.z))
        self.update()

        # State Machine
        if self.state=="GTG":
            self.go_straight()
        elif self.state=="KNN":
            self.knn()
        elif self.state=="RTURN":
            self.rturn()
        elif self.state=="LTURN":
            self.lturn()
        elif self.state=="UTURN":
            self.uturn()
        elif self.state=="LOST":
            self.find_wall()   
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

