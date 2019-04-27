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
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped


class Localize():
    def __init__(self, poseSt):
        '''
        poseSt: A PoseStamped message for the initial location
                of robot_base. Position should be in the form
                x,y,z (m). Orientation should be in euler
                notation (rad)
        output: Publishes the pose of robot_base at specified frequency
        '''
        self.pos = DeadReckoning(starting_pose)
        
        self.pos_pub = rospy.Publisher('/robot_base', PoseStamped, queue_size = 10)
        self.vel_sub = rospy.Subscriber('/command_vel', TwistStamped, self.vel_callback)
        
        self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.pos_pub.publish(self.pos.poseSt)

    def vel_callback(self, twistSt):
        self.pos.update(twistSt)


if __name__ == '__main__':
    rospy.init_node('localization_node')
    # set up an initial position
    s = Pose()
    s.position.x = 2.25
    s.position.y = 0.3
    s.orientation.z = PI/2
    localizer = Localize(s)
