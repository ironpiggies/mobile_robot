#!/usr/bin/env python

from paths import *
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped


class PathPlanner():
    
    def __init__(self):
        self.paths = {0: PATH_A,
                      1: PATH_B,
		      2: PATH_C}
        self.state = 0
        self.path_pub = rospy.Publisher('/path', Marker, queue_size=1)
        self.pos_sub = rospy.Subscriber('/robot_base', PoseStamped, self.posCallback)
        
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

    def posCallback(self, poseSt):
        pos_x = poseSt.pose.position.x
        pos_y = poseSt.pose.position.y
        target_x = self.paths[self.state][-1][0]
        target_y = self.paths[self.state][-1][1]
        if (pos_x-target_x)**2 + (pos_y-target_y)**2 < 0.07**2:
            self.state += 1
	if self.state == 3:
	    self.state = 0


if __name__=='__main__':
    rospy.init_node('path_planner')
    planer = PathPlanner()
    rospy.spin()
