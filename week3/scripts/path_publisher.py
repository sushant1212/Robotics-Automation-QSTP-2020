#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point


rospy.init_node('publisher')

pub = rospy.Publisher('/path', Point, queue_size=1)
p = Point()
p.x = 5.0
p.y = 5.0
p.z = 0.0
rate = rospy.Rate(10)


while not rospy.is_shutdown():
    pub.publish(p)
    rate.sleep()