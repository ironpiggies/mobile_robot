#!/usr/bin/env python

#-------------------------------------------------#
#                                                 #
# This file creates a node that broadcasts the    #
# tf transform of the robot_base relation to the  #
# map. The node listens to the '/robot_base'      #
# topic and broadcasts whenever a new PoseStamped #
# message is published there                      #
#                                                 #
#-------------------------------------------------#

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def broadcast_pose(msg):
    '''
    msg:    a PoseStamped message where position is in x,y,z (meters) and 
            orientation is in euler (rad)
    output: broadcasts msg as a transfrom from map frame to robot_base frame
    '''
    br = tf.TransformBroadcaster()
    x,y,z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    quatern = tf.transformations.quaternion_from_euler(0,0,msg.pose.orientation.z)
    br.sendTransform((x,y,z), quatern, msg.header.stamp, 'robot_base', 'map')

if __name__ == '__main__':
    rospy.init_node('robot_base_broadcaster')
    rospy.Subscriber('/robot_base', PoseStamped, broadcast_pose)
    rospy.spin()
