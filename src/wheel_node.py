#!/usr/bin/python

#------------------------------------------------#
#                                                #
# This node is based on the solution to Lab3.    #
# The node sends /command_vel to the arduino and #
# publishes /measured_vel based on response from #
# the Arduino.                                   #
#                                                #
#------------------------------------------------#


import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import TwistStamped
import helper
import numpy as np

serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)

MAX_WHEEL_VEL = 0.35
def cmdvel_callback(twistSt):
    '''
    twistSt:    A TwistStamped message of the desired velocity in the
                frame of robot_base. Given physical constraints, all
                values are assumed to be zero except linear.x and angular.z
    output:     Sends the desired velocity to the Arduino controller.
    '''
    twist = twistSt.twist
    # If turning...
    print twist
    if twist.angular.z != 0:
	b = 0.23
        tr = twist.linear.x / twist.angular.z
        if tr != 0:
	    print tr
	    v_L = twist.angular.z * (tr - b*np.sign(tr))
            v_R = twist.angular.z * (tr + b*np.sign(tr))
	else:
	    v_L = -1.0 * twist.angular.z * b
	    v_R = twist.angular.z * b    
    # If straight...
    else:
	v_L = twist.linear.x
        v_R = twist.linear.x

    # Limit max wheel velocity, keeping ratio the same
    scale = min(MAX_WHEEL_VEL / max(abs(v_L), abs(v_R)), 1.0)
    v_L = v_L*scale
    v_R = v_R*scale

    strCmd =  str(v_L) + ',' + str(v_R) + '\n'
    print strCmd
    serialComm.write(strCmd)


def publish_vel():
    '''
    Constantly communicates with the Arduino controller and publishes
    the measured speed of the wheels.
    '''
    velPub = rospy.Publisher('/measured_vel', TwistStamped, queue_size=1)
    while not rospy.is_shutdown():
        # get a string that represents current velocity from serial
        serialData = serialComm.readline()

        # split the string with commas
        splitData = serialData.split(',')

        # parse the 2 split strings into 2 floats
        try:
	    b = 0.23
            v_L = float(splitData[0])
            v_R = float(splitData[1])

            vel = TwistStamped()
            vel.twist.linear.x = (v_L+v_R)/2
            if (abs(v_L) < abs(v_R)):
                tr = abs((v_L+v_R)*b / (v_L-v_R))
                vel.twist.angular.z = vel.twist.linear.x / tr
            elif (abs(v_L) > abs(v_R)):
                tr = -1.0 * abs((v_L+v_R)*b / (v_L-v_R))
                vel.twist.angular.z = vel.twist.linear.x / tr
            else:
                vel.twist.angular.z = 0.0
            vel.header.stamp = rospy.Time()
            velPub.publish(vel)

        except:
            print 'Cannot parse ', splitData
            

if __name__=='__main__':
    rospy.init_node('wheels', anonymous=True)
    vel_thread = threading.Thread(target = publish_vel)
    vel_thread.start()
    rospy.Subscriber('/command_vel', TwistStamped, cmdvel_callback)
    rospy.spin()
