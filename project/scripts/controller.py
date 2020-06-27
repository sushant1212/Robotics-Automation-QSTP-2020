#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from tf.transformations import euler_from_quaternion
from math import pi, atan2, sqrt

class Robot():

    def __init__(self):
        self.sub1 = rospy.Subscriber('/path', Path, self.callback1)
        self.sub2 = rospy.Subscriber('/odom', Odometry, self.callback2)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal = Point() #temporary goals
        self.goal_angle = None #amount of angle to rotate
        self.coordinates = Odometry()
        self.angle = None #angle woth positive X axis
        self.flag = 0 #to check if the bot has finished rotating
        self.count1 = 0 #count variables are to run the initialization part of the pid only once
        self.count2 = 0
        self.count3 = 0 #this count variable is to run the initialization of the goal step
        self.start = Point() #this is to get the starting coordinates of the bot when it receives a new goal
        self.goal_step = None
        self.final_goal = (6,6)  #change this accordingly
        self.total_points = None
        #terms for pid-angle
        self.pid_angle_e_sum = 0
        self.pid_angle_e_prev = None

        #terms for pid-coordinate
        self.pid_coord_e_sum = 0
        self.pid_coord_e_prev = None

    
    def pid_angle(self, goal_angle, K_P, K_I, K_D):
        e = goal_angle - self.angle
        self.pid_angle_e_sum += e
        dedt = e - self.pid_angle_e_prev
        t = Twist()
        t.angular.z = K_P*e + K_D*dedt + K_I*self.pid_angle_e_sum
        self.pub.publish(t)
        self.pid_angle_e_prev = e

    def pid_coord(self, K_P, K_D, K_I):
        coord = self.coordinates.pose.pose.position
        e = self.dist(self.goal.x, self.goal.y,self.start.x, self.start.y) - self.dist(coord.x, coord.y, self.start.x, self.start.y)
        self.pid_coord_e_sum += e
        dedt = e - self.pid_coord_e_prev
        t = Twist()
        t.linear.x = K_P*e + K_D*dedt + K_I*self.pid_angle_e_sum
        self.pub.publish(t)
        self.pid_coord_e_prev = e


    def get_euler_angle(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw


    def dist(self, x1, y1, x2, y2):
        return sqrt((x1 - x2)**2 + (y1-y2)**2)


    def callback1(self, msg):
        points = []
        l = msg.poses
        for i in range(len(l)):
            a = l[i]
            x = a.pose.position.x
            y = a.pose.position.y
            points.append((x,y))
        self.total_points = len(points)
        
        if self.count3 == 0:
            self.goal_step = len(points) - 2 #last entry in the list is the starting point i.e origin
            self.count3 += 1
        
        self.goal.x = points[self.goal_step][0]
        self.goal.y = points[self.goal_step][1]

        if self.goal_step < 0:
            self.goal.x = self.final_goal[0]
            self.goal.y = self.final_goal[1]
        
        if self.goal_step < -1:
            print('Reached GOAL!!!!!!!!')

        

    
    def callback2(self, msg):
        coord = self.coordinates.pose.pose.position
        coord.x = msg.pose.pose.position.x
        coord.y = msg.pose.pose.position.y
        coord.z = msg.pose.pose.position.z
        self.goal_angle = atan2(self.goal.y - coord.y, self.goal.x - coord.x)
        self.angle = self.get_euler_angle(msg)


        print(self.goal.x, self.goal.y)

        if self.count1 == 0:
            self.pid_angle_e_prev = self.goal_angle - self.angle
            self.count1 += 1

        if self.goal_angle - self.angle > 0.1 or self.goal_angle - self.angle < -0.1:
            self.pid_angle(self.goal_angle, 0.3, 0.0, 0.0)
        
        else:
            self.flag = 1 #rotation complete
            t = Twist()
            t.angular.z = 0
            self.pub.publish(t)
        
        if self.flag == 1:
            if self.count2 == 0:
                self.pid_coord_e_prev = self.dist(self.goal.x, self.goal.y, coord.x, coord.y)
                self.count2 += 1

            if self.dist(self.goal.x, self.goal.y, coord.x, coord.y) > 0.1:
                self.pid_coord(0.3, 0.001, 0.0)

            else:
                t = Twist()
                t.linear.x = 0
                self.pub.publish(t)
                self.pid_angle_e_sum = 0
                self.pid_angle_e_prev = None
                self.pid_coord_e_sum = 0
                self.pid_coord_e_prev = None
                self.goal_step -= 1
                self.flag = 0
                self.count1 = 0
                self.count2 = 0
                self.start.x = coord.x
                self.start.y = coord.y
                
if __name__ == "__main__":
    rospy.init_node('mover')
    robot = Robot()
    rospy.spin()