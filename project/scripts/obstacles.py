#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

obstacles = [(0, 1.5), (0, 3), (0, 4.5), (1.5, 0), (1.5, 1.5), (1.5, 3), (1.5,4.5), (3,0), (3,1.5), (3,3), (3,4.5),(4.5,0), (4.5, 1.5), (4.5,3), (4.5,4.5)]
rospy.init_node('obstacle_detector')
pub = rospy.Publisher('/obstacles', Path, queue_size=10)
path = Path()
for i in range(len(obstacles)):
    pose = PoseStamped()
    pose.pose.position.x = obstacles[i][0]
    pose.pose.position.y = obstacles[i][1]

    path.poses.append(pose)

rate = rospy.Rate(15)

while not rospy.is_shutdown():
    pub.publish(path)
    rate.sleep()