#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def broadcast_pose(msg):
    br = tf.TransformBroadcaster()
    x,y,z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    quatern = tf.transformations.quaternion_from_euler(0,0,msg.pose.orientation.z)
    br.sendTransform((x,y,z), quatern, msg.header.stamp, 'robot_base', 'map')

if __name__ == '__main__':
    rospy.init_node('robot_base_broadcaster')
    rospy.Subscriber('/robot_base', PoseStamped, broadcast_pose)
    rospy.spin()

