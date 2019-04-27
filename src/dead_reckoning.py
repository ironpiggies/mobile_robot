#------------------------------------------------#
#                                                #
# This file defines the DeadReckoning Class. The #
# class maintains and modifies a pose based on   #
# velocity measurements and the time between     #
# updates.                                       #
#                                                #
#------------------------------------------------#


import tf
import rospy
from numpy import sin, cos
from geometry_msgs.msg import PoseStamped, TwistStamped


class DeadReckoning():
    def __init__(self, poseSt):
        '''
        poseSt: A PoseStamped message. The positions should be in
                x,y,z form (m) and the angles should be in euler
                notation (rad)
        output: Initializes position and orientation
        '''
        self.x = poseSt.pose.position.x
        self.y = poseSt.pose.position.y
        self.theta = poseSt.pose.orientation.z
        self.update_time = rospy.get_time()
        self.poseSt = PoseStamped()

    def update(self, twistSt):
        '''
        twistSt:    A TwistStamped message in the frame of robot_base.
                    Given the physical constraints, the only nonzero
                    values are assumed to be linear.x and angular.z
        output:     Updates x, y, theta, and poseSt
        '''
        dt = rospy.get_time() - self.update_time
        self.update_time = rospy.get_time()
        twist = twistSt.twist
        # if no rotation...
        if msg.angular.z==0:
            dx = dt * twist.linear.x
            self.x += dx*cos(self.theta)
            self.y += dx*sin(self.theta)
        # if there is rotation...
        else:
            r = twist.linear.x / twist.angular.z
            dtheta = dt*twist.angular.z
            dx = r*np.sin(dtheta)
            dy = r*(1-np.cos(dtheta))
            self.x += dx*np.cos(self.theta) - dy*np.sin(self.theta)
            self.y += dx*np.sin(self.theta) + dy*np.cos(self.theta)
            self.theta += dtheta
        
        self.poseSt.header.stamp = rospy.Time()
        self.poseSt.pose.position.x = self.x
        self.poseSt.pose.position.y = self.y
        self.poseSt.pose.orientation.z = self.theta

    def reset(self, poseSt):
        '''
        poseSt: A pose message for resetting the position. The positions
                should be in x,y,z form (m) and the angles should be in
                euler notation (rad)
        output: Resets the position to the given pose
        '''
        self.x = poseSt.pose.position.x
        self.y = poseSt.pose.position.y
        self.theta = poseSt.pose.orientation.z
        self.poseSt = poseSt
