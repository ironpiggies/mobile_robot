#!/usr/bin/env python
from dead_reckoning import DeadReckoning
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped

class Localize():
    def __init__(self, starting_pose):
        self.pos = DeadReckoning(starting_pose)
        
        self.pos_pub = rospy.Publisher('/robot_base', PoseStamped, queue_size = 10)
        self.vel_sub = rospy.Subscriber('/command_vel', TwistStamped, self.vel_callback)
        
    def vel_callback(self, msg):
        print 'check 1'
        self.pos.update(msg.twist)
        print 'check 2'
        self.pos_pub.publish(self.pos.poseStamped)
        print 'check 3'


if __name__ == '__main__':
    rospy.init_node('localization_node')
    s = Pose()
    s.position.x = 3.5
    s.position.y = 0.3
    s.orientation.z = np.pi/2

    localizer = Localize(s)
    rospy.spin()
