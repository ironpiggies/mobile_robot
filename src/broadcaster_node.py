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

    #Nic's code added for april tags. feel free to comment out if it doesn't work!
    lr=tf.TransformListener()
    apriltag_est=lookupTransform(lr,'/robot_base_est','/map')
    if apriltag_est: #if there is
        #apply averaging
        x_at,y_at,z_at,wx_at,wy_at,wz_at=apriltag_est


        at_conf=.5 #represents how confident we are in april tags relative to particle filter.

        wz_est=(1-at_conf)*msg.pose.orientation.z+at_conf*wz_at
        x=(1-at_conf)*x+at_conf*x_at #weighted but total weight is 1
        y=(1-at_conf)*y+at_conf*y_at
        z=(1-at_conf)*z+at_conf*z_at
        quatern=tf.transformations.quaternion_from_euler(0,0,wz_est)
    ### end of my code



    br.sendTransform((x,y,z), quatern, msg.header.stamp, 'robot_base', 'map')

if __name__ == '__main__':
    rospy.init_node('robot_base_broadcaster')
    rospy.Subscriber('/robot_base', PoseStamped, broadcast_pose)
    rospy.spin()
