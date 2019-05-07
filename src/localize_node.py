#!/usr/bin/env python

#-----------------------------------------------#
#                                               #
# This file creates a node for publishing the   #
# current pose of the robot to /robot_base. The #
# current design only implements dead reckoning #
# but should be extended to include an option   #
# for localizing with the particle filter.      #
#                                               #
#-----------------------------------------------#


from dead_reckoning import DeadReckoning
from numpy import pi as PI
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
import tf

class Localize():
    def __init__(self, poseSt):
        '''
        poseSt: A PoseStamped message for the initial location
                of robot_base. Position should be in the form
                x,y,z (m). Orientation should be in euler
                notation (rad)
        output: Publishes the pose of robot_base at specified frequency
        '''
        self.pos = DeadReckoning(poseSt)

        self.pos_pub = rospy.Publisher('/robot_base', PoseStamped, queue_size = 10)
        self.vel_sub = rospy.Subscriber('/measured_vel', TwistStamped, self.vel_callback)
        self.reset_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.reset)
        self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.pos_pub.publish(self.pos.poseSt)
            self.rate.sleep()

    def vel_callback(self, twistSt):
        self.pos.update(twistSt)

    def reset(self, poseCovSt):
        poseSt = PoseStamped()
        poseSt.pose = poseCovSt.pose.pose
        orientation = poseCovSt.pose.pose.orientation
        q = (0,0,orientation.z, orientation.w)
        poseSt.pose.orientation.z = tf.transformations.euler_from_quaternion(q)[2]
        self.pos.reset(poseSt)


if __name__ == '__main__':
    rospy.init_node('localization_node')
    # set up an initial position
    s = PoseStamped()
    s.pose.position.x = 2.1
    s.pose.position.y = 0.4
    s.pose.orientation.z = PI/2
    s.header.stamp = rospy.Time()
    localizer = Localize(s)
