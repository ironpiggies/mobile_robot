#!/usr/bin/env python

from paths import PATH_A
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PathPlanner():
    
    def __init__(self):
        self.paths = {'A': PATH_A}
        self.state = 'A'
        self.path_pub = rospy.Publisher('/path', Marker, queue_size=1)
        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            path = self.paths[self.state]
            self.publish_PoseArray(path)
            self.rate.sleep()

    def publish_PoseArray(self, path):
        pts = Marker()
        pts.header.frame_id = 'map'
        pts.scale.x, pts.scale.y = 0.1,0.1
        pts.color.a = 1.0
        pts.color.g = 1.0
        pts.type = 7 # 8 indicates its a list of points
        for pt in path:
            p = Point()
            p.x, p.y = pt[0], pt[1]
            pts.points.append(p)
        self.path_pub.publish(pts) 

if __name__=='__main__':
    rospy.init_node('path_planner')
    planer = PathPlanner()
    rospy.spin()
