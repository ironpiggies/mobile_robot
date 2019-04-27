import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Pose, TwistStamped

class DeadReckoning():
    def __init__(self, starting_pose):
        self.x = starting_pose.position.x
        self.y = starting_pose.position.y
        self.theta = starting_pose.orientation.z
        self.update_time = rospy.get_time()
        self.poseStamped = PoseStamped()

    def update(self, msg):
        dt = rospy.get_time() - self.update_time
        self.update_time = rospy.get_time()
        # if no rotation...
        if msg.angular.z==0:
            dx = dt * msg.linear.x
            self.x += dx*np.cos(self.theta)
            self.y += dx*np.sin(self.theta)
        # if there is rotation...
        else:
            r = msg.linear.x / msg.angular.z
            dtheta = dt*msg.angular.z
            dx = r*np.sin(dtheta)
            dy = r*(1-np.cos(dtheta))
            self.x += dx*np.cos(self.theta) - dy*np.sin(self.theta)
            self.y += dx*np.sin(self.theta) + dy*np.cos(self.theta)
            self.theta += dtheta
        
        self.poseStamped.header.stamp = rospy.Time()
        self.poseStamped.pose.position.x = self.x
        self.poseStamped.pose.position.y = self.y
        self.poseStamped.pose.orientation.z = self.theta

    def reset(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        angles = tf.transformation.euler_from_quaternion(msg.orientation)
        self.theta = angles[2]

