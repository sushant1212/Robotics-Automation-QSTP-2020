#!/usr/bin/env python
import matplotlib.pyplot as plt
from scipy.spatial import distance
import random
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path



class Point():

    points_list = []
    instances = []
    dist = 0.5
    obstacles = []
    #obstacles = [(0, 1.5), (0, 3), (0, 4.5), (1.5, 0), (1.5, 1.5), (1.5, 3), (1.5,4.5), (3,0), (3,1.5), (3,3), (3,4.5),(4.5,0), (4.5, 1.5), (4.5,3), (4.5,4.5)]
    def __init__(self, x, y, nn_x=None, nn_y=None):
        self.x = x
        self.y = y
        self.nn_x = nn_x
        self.nn_y = nn_y
        Point.points_list.append((self.x, self.y))
    
    @staticmethod
    def nearest_neigbour(x,y):
        l = len(Point.points_list)
        d = distance.euclidean([x,y], [Point.points_list[0][0],Point.points_list[0][1]])
        nn_x = Point.points_list[0][0]
        nn_y = Point.points_list[0][1]
        for i in range(1,l):
            dp = distance.euclidean([x,y], [Point.points_list[i][0],Point.points_list[i][1]])
            if(dp < d):
                d = dp
                nn_x = Point.points_list[i][0]
                nn_y = Point.points_list[i][1]
            else:
                pass
        return nn_x, nn_y

    @staticmethod
    def plot(x,y, color):
        plt.scatter(x,y,marker='o', color=str(color))

    @classmethod
    def generate_random_point(cls):
        p_x = 6*random.random()
        p_y = 6*random.random()
        nn_x, nn_y = Point.nearest_neigbour(p_x, p_y)

        x_new, y_new = Point.generate_point(Point.dist, nn_x, nn_y, p_x, p_y)

        for i in range(len(Point.points_list)):
            l = Point.points_list
            if distance.euclidean([x_new, y_new], [l[i][0], l[i][1]]) < 0.5:
                return 0
        
        for i in range(len(Point.obstacles)):
            l = Point.obstacles
            if distance.euclidean([x_new, y_new], [l[i][0], l[i][1]]) < 0.7:
                return 0

        Point.plot(x_new, y_new, 'b')
        plt.plot([nn_x, x_new], [nn_y, y_new], color = 'y')

        return cls(x_new, y_new, nn_x, nn_y)

    @staticmethod
    def generate_point(dist, x1,y1,x2,y2):
        d = distance.euclidean([x1,y1], [x2,y2])
        m = float(dist)
        n = float(d - dist)
        x_new = (n*x1+m*x2)/(m+n)
        y_new = (n*y1+m*y2)/(m+n)
        return x_new, y_new

class Subscriber():
    def __init__(self):
        self.sub = rospy.Subscriber('/obstacles', Path, self.callback)
        self.count = 0
    
    def callback(self, msg):
        if self.count == 0:
            l = msg.poses
            for i in range(len(l)):
                a = l[i]
                x = a.pose.position.x
                y = a.pose.position.y
                Point.obstacles.append((x,y))
            self.count += 1
        else:
            pass



if __name__ == "__main__":
    rospy.init_node('path_planner')
    sub = Subscriber()


    r = rospy.Rate(1)
    r.sleep()
    fig, ax = plt.subplots()
    for i in range(len(Point.obstacles)):
        o = Point.obstacles
        a = plt.Circle((o[i][0], o[i][1]), 0.25, color='k')
        ax.add_artist(a) 
    #Defining and plotting goal and start point
    start = (0,0)
    goal = (6,6)
    plt.scatter(start[0], start[1], marker='o', color='r')
    plt.scatter(goal[0], goal[1], marker='o', color='g')


    start = Point(start[0],start[1]) # so that the start point is appended to the points list
    parent = {}


    flag = 0
    goal_nn_x = None
    goal_nn_y = None
    count = 0


    while(True):
        #OPTIMIZE THIS FOR LOOP CHECK
        for i in range(len(Point.points_list)):
            l = Point.points_list
            x = l[i][0]
            y = l[i][1]
            
            if distance.euclidean([goal[0], goal[1]], [x,y]) < 1 and distance.euclidean([goal[0], goal[1]], [x,y]) > 0.4:
                flag = 1
                goal_nn_x = x
                goal_nn_y = y 
        

        if flag == 0:
            v = Point.generate_random_point()
            if v == 0:
                pass
            else:
                parent['('+str(v.x)+','+str(v.y)+')'] = (v.nn_x, v.nn_y)
        
        else:
            break

    plt.plot([goal[0], goal_nn_x], [goal[1], goal_nn_y], color='r')



    
    path = Path()
    pose = PoseStamped()
    x = goal_nn_x
    y = goal_nn_y
    nn_x = None
    nn_y = None

    pose.pose.position.x = x
    pose.pose.position.y = y

    path.poses.append(pose)


    while(x != start.x and y != start.y):
        (nn_x, nn_y) = parent['('+str(x)+','+str(y)+')'] 
        plt.plot([nn_x, x], [nn_y, y], color = 'r')
        x = nn_x
        y = nn_y
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)

    pub = rospy.Publisher('/path', Path, queue_size=10)

    plt.show()
    print('No. of points :'+ str(len(path.poses)))
    print('planned path!')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(path)
        
        rate.sleep()   
    
    