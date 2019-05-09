#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

class WaiterLocator():
    
    def __init__(self):
	self.robot_x = 0.0
	self.robot_y = 0.0
	self.robot_theta = 0.0
	self.waiter = True
	self.m_to_px = 100
	self.waiter_pub = rospy.Publisher('/waiter', Bool, queue_size=1)
	self.pos_sub = rospy.Subscriber('/robot_base', PoseStamped, self.posCallback)
        self.slice_sub = rospy.Subscriber('/waiter_slice', Marker, self.cameraCallback)

    def posCallback(self, poseSt):
	self.robot_x = poseSt.pose.position.x
	self.robot_y = poseSt.pose.position.y
	self.robot_theta = poseSt.pose.orientation.z

    def cameraCallback(self, msg):
    	'''
    	Updates the location of the waiter
    	'''
    	points = []
    	for p in msg.points:
	    points.append([p.x, p.y])
        if len(points) == 0:
	    return 	
	pts = np.array(points)
    
    	# convert points to global frame
    	points = np.zeros(pts.shape)
    	points[:,0] = pts[:,0]*np.cos(self.robot_theta) - pts[:,1]*np.sin(self.robot_theta) + self.robot_x
    	points[:,1] = pts[:,0]*np.sin(self.robot_theta) + pts[:,1]*np.cos(self.robot_theta) + self.robot_y
    	# keep points where waiter could be
    	points = points[np.logical_and(points[:,0]>0.05, points[:,0]<0.95),:]
    	points = points[np.logical_and(points[:,1]>1.3, points[:,1]<1.7)]
 	# if no valid points, no waiter
	if points.shape[0]==0:
	    return
    	# average x values 
	else:
    	    x_mean = np.mean(points[:,0])
    	    if x_mean > 0.5:
                self.waiter = True
    	    else:
                self.waiter = False

    	w = Bool()
    	w.data = self.waiter
    	self.waiter_pub.publish(w)

if __name__ == '__main__':
    rospy.init_node('waiter_locator')
    loc = WaiterLocator()
    rospy.spin()
