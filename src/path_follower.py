#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

class SimplePursuit():
    def __init__(self):
        self.v = 0.2
        self.look_ahead = 4
        self.x = None
        self.y = None
        self.theta = None
        self.stop_dist = 0.05
	self.wait = False
        self.path = []

        self.vel_pub = rospy.Publisher('/command_vel', TwistStamped, queue_size=1)
        self.pos_sub = rospy.Subscriber('/robot_base', PoseStamped, self.posCallback)
        self.path_sub = rospy.Subscriber('/path', Marker, self.pathCallback)
	self.waiter_sub = rospy.Subscriber('/waiter', Bool, self.waiterCallback)
        self.rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.x != None:
                self.pubVelocity()
            self.rate.sleep()

    def waiterCallback(self, msg):
	if msg.data:
	    if self.x<1.2 and self.x>0.3:
		if self.y<1.8 and self.y>1.2:
		    self.wait = True
		    #return
	self.wait = False
	

    def pubVelocity(self):
        if len(self.path)==0 or self.wait:
            self.vel_pub.publish(TwistStamped())
            return
        # find nearest point
        dist = [(self.x - p[0])**2 + (self.y - p[1])**2 for p in self.path]
        target_ind = min(np.argmin(dist) + self.look_ahead, len(self.path)-1)
        target = self.path[target_ind]
        # reorient target into robot_base frame
        dx = target[0] - self.x
        dy = target[1] - self.y
        rx = dx*np.cos(self.theta) + dy*np.sin(self.theta)
        ry = -dx*np.sin(self.theta) + dy*np.cos(self.theta)
        vel = TwistStamped()
       
        # publish no velocity if at the end of the path
        if rx**2 + ry**2 < self.stop_dist**2 and target_ind==len(self.path)-1:
            self.vel_pub.publish(vel)
            return

        # allow for forward or reverse driving
        vel.twist.linear.x = self.v * np.sign(rx)
        # if ry==0, drive straight
        # otherwise, calculate turn
        if ry != 0:
            R = (rx**2+ry**2)/(2*ry)
            vel.twist.angular.z = self.v / R * np.sign(rx)
	# if the target is less than 45 degress behind you, do not drive backwards
	ang = np.arctan2(ry, rx)
	if vel.twist.linear.x < 0.0 and abs(ang) < 3.0/4.0*np.pi:
	    vel.twist.linear.x = -1.0*vel.twist.linear.x
	    vel.twist.angular.z = -1.0*vel.twist.linear.x
        
	self.vel_pub.publish(vel)

    def posCallback(self, poseSt):
        self.x = poseSt.pose.position.x
        self.y = poseSt.pose.position.y
        self.theta = poseSt.pose.orientation.z

    def pathCallback(self, mkr):
        self.path = []
        for pt in mkr.points:
            self.path.append((pt.x, pt.y))


if __name__=='__main__':
    rospy.init_node('Simple_Pursuit')
    pursuit = SimplePursuit()
    rospy.spin()

