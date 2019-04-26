#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('command_vel', Twist, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd = Twist()
	cmd.linear.x = 0.0
	cmd.angular.z = 0.25
	pub.publish(cmd)
	rate.sleep()
if __name__== '__main__':
    try:
	talker()
    except rospy.ROSInterruptException:
	pass
