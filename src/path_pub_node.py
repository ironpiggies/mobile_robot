#!/usr/bin/env python

from paths import *
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Bool

class PathPlanner():
    
    def __init__(self):
        self.paths = {0: PATH_STOP,
                      1: PATH_0,
		      2: PATH_1A,
		      3: PATH_STOP,
		      4: PATH_2,
		      5: PATH_3,
		      6: PATH_STOP,
		      7: PATH_4,
		      8: PATH_5,
		      9: PATH_STOP}
        self.state = 0
	self.start_wait_time = 0.0
	self.need_to_set_time = True
	self.switch_dist = 0.07
	self.waiter = True
	self.start = True ### !!! Need to set to false for communication
	self.pizza_transfered = True ### !!! Need to set to false for communication
        self.path_pub = rospy.Publisher('/path', Marker, queue_size=1)
        self.pos_sub = rospy.Subscriber('/robot_base', PoseStamped, self.posCallback)
	self.com_sub = rospy.Subscriber('tcp_command', String, self.tcpCallback)
	self.waiter_sub = rospy.Subscriber('/waiter', Bool, self.waiterCallback)        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            path = self.paths[self.state]
            self.publish_PoseArray(path)
            self.rate.sleep()

    def waiterCallback(self, msg):
	self.waiter = msg.data

    def tcpCallback(self, msg):
	if msg == 'Start':
	    self.start = True
	if msg == 'Transfered':
	    self.pizza_transfered = True

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
	
	if self.state==0:
	    if self.start == True:
		self.state = 1
	    return
		
	if self.state==1:
	    target_x = self.paths[1][-1][0]
	    target_y = self.paths[1][-1][1]
	    if (pos_x-target_x)**2 + (pos_y-target_y)**2 < self.switch_dist**2:
            	self.state = 2
		# pick a path to follow
		if not self.waiter:
		    self.paths[2] = PATH_1A
		else:
		    self.paths[3] = PATH_1B
	
	if self.state==2:
	    x = self.paths[2][-1][0]
	    y = self.paths[2][-1][1]
	    if (pos_x-x)**2 + (pos_y-y)**2 < self.switch_dist**2:
		self.state = 3

	if self.state==3:
	    if self.need_to_set_time:
		self.start_wait_time = rospy.get_time()
		self.need_to_set_time = False
	    if rospy.get_time() - self.start_wait_time > 2.0:
		self.state = 4
		self.need_to_set_time = True
	
	if self.state==4:
	    target_x = self.paths[4][-1][0]
	    target_y = self.paths[4][-1][1]
	    if (pos_x-target_x)**2 + (pos_y-target_y)**2 < self.switch_dist**2:
		self.state = 5
	
	if self.state==5:
	    target_x = self.paths[5][-1][0]
	    target_y = self.paths[5][-1][1]
	    if (pos_x-target_x)**2 + (pos_y-target_y)**2 < self.switch_dist**2:
		# TELL DELTA THAT YOU ARE READY FOR THE PIZZA HERE
		self.state = 6
	
	if self.state == 6:
	    if self.pizza_transfered:
		self.state = 7
		return
	
	if self.state == 7:
	    target_x = self.paths[7][-1][0]
	    target_y = self.paths[7][-1][1]
	    if (pos_x-target_x)**2 + (pos_y-target_y)**2 < self.switch_dist**2:
		self.state = 8

	if self.state == 8:
	    target_x = self.paths[8][-1][0]
	    target_y = self.paths[8][-1][1]
	    if (pos_x-target_x)**2 + (pos_y-target_y)**2 < self.switch_dist**2:
	        self.state = 8

	# just stay in state 9

if __name__=='__main__':
    rospy.init_node('path_planner')
    planer = PathPlanner()
    rospy.spin()
