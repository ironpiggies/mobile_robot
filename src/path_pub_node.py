#!/usr/env/bin python

from paths import PATH_A
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PathPlanner():
    
    def __init__(self):
        self.paths = {'A': PATH_A}
        self.state = 'A'
        self.path_pub = rospy.Publisher('/path', PoseArray, queue_size=1)
        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            path = self.paths[self.state]
            self.publish_PoseArray(path)
            self.rate.sleep()

    def publish_PoseArray(path)
        pts = Marker()
        pts.type = 8 # 8 indicates its a list of points
        for pt in path:
            p = Point()
            p.x, p.y = pt[0], pt[1]
            pts.Points.append(p)
        self.path_pub  
