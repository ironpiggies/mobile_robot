#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from visualization_msgs.msg import Marker

class SimplePursuit():
    def __init__(self):
        self.v = 0.1
        self.look_ahead = 2
        self.x = None
        self.y = None
        self.theta = None
        self.stop_dist = 0.05
        self.path = []

        self.vel_pub = rospy.Publisher('/command_vel', TwistStamped, queue_size=1)
        self.pos_sub = rospy.Subscriber('/robot_base', PoseStamped, self.posCallback)
        self.path_sub = rospy.Subscriber('/path', Marker, self.pathCallback)

        self.rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.x != None:
                self.pubVelocity()
            self.rate.sleep()


    def pubVelocity(self):
        if len(self.path)==0:
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
        
        if rx**2 + ry**2 < self.stop_dist and target_ind==len(self.path)-1:
            self.vel_pub.publish(vel)
            return

        vel.twist.linear.x = self.v
        # if ry==0, drive straight
        # otherwise, calculate turn
        if ry != 0:
            R = (rx**2+ry**2)/(2*ry)
            vel.twist.angular.z = self.v / R

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

