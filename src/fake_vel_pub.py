#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped

def talker():
    pub = rospy.Publisher('command_vel', TwistStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd = TwistStamped()
	cmd.twist.linear.x = 0.3
	cmd.twist.angular.z = 1.0
	cmd.header.stamp = rospy.Time()
        pub.publish(cmd)
	rate.sleep()
if __name__== '__main__':
    try:
	talker()
    except rospy.ROSInterruptException:
	pass
