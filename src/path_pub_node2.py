#!/usr/bin/env python

from paths2 import *
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String

class PathPlanner():
    
    def __init__(self):
        self.paths = {0: PATH_STOP,
                      1: PATH_A,
		      2: PATH_B,
		      3: PATH_C1,
		      4: PATH_STOP,
		      5: PATH_D,
		      6: PATH_E,
		      7: PATH_F,
		      8: PATH_G1,
		      9: PATH_STOP,
		      10: PATH_H}
        self.state = 0
	self.start_wait_time = 0.0
	self.waiter = "RIGHT"
	self.waiter_counts = 0
	self.soda_delivered = False
	self.need_to_set_time = True
	self.switch_dist = 0.07
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
	if msg.data:
	    if self.waiter == "RIGHT":
		self.waiter_counts += 1
	else:
	    if self.waiter == "LEFT":
		self.waiter_counts += 1
	if self.waiter_counts > 5:
	    if msg.data:
		self.waiter = "LEFT"
	    else:
		self.waiter = "RIGHT"
	    self.waiter_counts = 0

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
	    if self.start:
		self.state = 1
	
	# move to in front of the waiter
	if self.state==1:
	    target_x = self.paths[1][-1][0]
	    target_y = self.paths[1][-1][1]
	    if (pos_x-target_x)**2 + (pos_y-target_y)**2 < self.switch_dist**2:
            	self.state = 2
	
	# get to neutral position
	if self.state==2:
	    if self.waiter = "RIGHT":
	        self.paths[3] = PATH_C1
		self.path_side = "LEFT"
	    else:
		self.paths[3] = PATH_C2
		self.path_side = "RIGHT"
	    x = self.paths[2][-1][0]
	    y = self.paths[2][-1][1]
	    if (pos_x-x)**2 - (pos_y-y)**2 < self.switch_dist**2:
	        self.state = 3
	
	# move past waiter, checking for change
	if self.state==3:
	    if self.waiter == self.path_side:
		# abort, go back
		self.state = 2
		return
	    x = self.paths[3][-1][0]
	    y = self.paths[3][-1][1]
	    if (pos_x-x)**2 + (pos_y-y)**2 < self.switch_dist**2:
		self.state = 4

	# wait for the soda to be picked up
	if self.state==4:
	    if self.need_to_set_time:
		self.start_wait_time = rospy.get_time()
		self.need_to_set_time = False
	    if rospy.get_time() - self.start_wait_time > 2.0:
		self.state = 5
		self.need_to_set_time = True
	    if self.soda_delivered:
		self.state = 4
	
	# drive backwards
	if self.state==5:
	    x = self.paths[5][-1][0]
	    y = self.paths[5][-1][1]
	    if (pos_x-x)**2 + (pos_y-y)**2 < self.switch_dist**2:
		self.state = 6

	# drive to back of waiter
	if self.state==6:
	    x = self.paths[6][-1][0]
	    y = self.paths[6][-1][1]
	    if (pos_x-x)**2 + (pos_y-y)**2 < self.switch_dist**2:
		self.state = 7

	# drive to neutral position behind waiter
	if self.state==7:
	    if self.waiter == "RIGHT":
		self.paths[8] = PATH_G1
		self.path_side = "LEFT"
	    else:
		self.paths[8] = PATH_G2
		self.path_side = "RIGHT"
	    x = self.paths[7][-1][0]
	    y = self.paths[7][-1][0]
	    if (pos_x-x)**2 + (pos_y-y)**2 < self.switch_dist**2:
		self.state = 8
	
	# drive past the waiter
	if self.state==8:
	    if self.path_side == self.waiter:
		self.state  = 7
	    x = self.paths[8][-1][0]
	    y = self.paths[8][-1][1]
	    if (pos_x-x)**2 + (pos_y-y)**2 < self.switch_dist**2:
		self.state = 9

	# wait for pizza
	if self.state==9:
	    if self.pizza_transfered:
		self.state = 10

	# back up
	if self.state==10:
	    x = self.paths[10][-1][0]
	    y = self.paths[10][-1][0]
	    if (pos_x-x)**2 + (pos_y-y)**2 < self.switch_dist**2:
		self.state = 2 # go back to first part again




if __name__=='__main__':
    rospy.init_node('path_planner')
    planer = PathPlanner()
    rospy.spin()
