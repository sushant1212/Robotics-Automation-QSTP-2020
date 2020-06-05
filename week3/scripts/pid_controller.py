#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Point
from tf.transformations import euler_from_quaternion
from math import pi, atan2, sqrt



class Bot():
    def __init__(self):
        self.sub1 = rospy.Subscriber('/path', Point, self.callback1)
        self.sub2 = rospy.Subscriber('/odom', Odometry, self.callback2)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal = Point()
        self.goal_angle = None
        self.coordinates = Odometry()
        self.angle = None
        self.flag = 0
        self.count1 = 0 
        self.count2 = 0
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
        e = self.dist(self.goal.x, self.goal.y, 0,0) - self.dist(coord.x, coord.y, 0, 0)
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
        self.goal.x = msg.x
        self.goal.y = msg.y
        self.goal.z = 0
    
    def callback2(self, msg):
        coord = self.coordinates.pose.pose.position
        coord.x = msg.pose.pose.position.x
        coord.y = msg.pose.pose.position.y
        coord.z = msg.pose.pose.position.z
        self.goal_angle = atan2(self.goal.y - coord.y, self.goal.x - coord.x)
        self.angle = self.get_euler_angle(msg)

        print(self.dist(self.goal.x, self.goal.y, coord.x, coord.y))


        if self.count1 == 0:
            self.pid_angle_e_prev = self.goal_angle - self.angle
            self.count1 += 1

        if self.goal_angle - self.angle > 0.01:
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

            if self.dist(self.goal.x, self.goal.y, coord.x, coord.y) > 0.15:
                self.pid_coord(0.02, 0.0, 0.0)

            else:
                t = Twist()
                t.linear.x = 0
                self.pub.publish(t)
                print('Done!')
                
if __name__ == "__main__":
    rospy.init_node('controller')
    bot = Bot()
    rospy.spin()