#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion, Twist
import helper
import numpy as np
serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)

def main():
    rospy.init_node('wheels', anonymous=True)
    
    #odometry_thread = threading.Thread(target = read_odometry_loop)
    #odometry_thread.start()
    vel_thread = threading.Thread(target = publish_vel)
    vel_thread.start()

    ## 1. Initialize a subscriber
    rospy.Subscriber('/command_vel', Twist, cmdvel_callback)
    
    rospy.spin()


def cmdvel_callback(msg):  
    ## 2. Turn the twist message into wheel velocities and send them to the arduino
    print 'Got a velocity command'
    if msg.angular.z != 0:
        print 'Calculating turn'
	b = 0.21 # distance between center of robot and wheels
        turning_radius = msg.linear.x / msg.angular.z
        if turning_radius != 0:
	    v_L = msg.angular.z * (turning_radius - b*np.sign(turning_radius))
            v_R = msg.angular.z * (turning_radius + b*np.sign(turning_radius))
	else:
	    v_L = -1.0 * msg.angular.z * b
	    v_R = msg.angular.z * b    
    else:
        print 'Going straight'
	v_L = msg.linear.x
        v_R = msg.linear.x

    strCmd =  str(v_L) + ',' + str(v_R) + '\n'
    print 'Writing wheel velocities ', strCmd
    serialComm.write(strCmd)
    


# read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
def read_odometry_loop():
    prevtime = rospy.Time.now()
    while not rospy.is_shutdown():
        # get a line of string that represent current odometry from serial
        serialData = serialComm.readline()
        
        # split the string e.g. "0.1,0.2,0.1" with cammas
        splitData = serialData.split(',')
        
        # parse the 3 split strings into 3 floats
        try:
            x     = float(splitData[0])
            y     = float(splitData[1])
            theta = float(splitData[2])
            
            hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
            prevtime = rospy.Time.now()
            
            print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz
            
            # publish odometry as Pose msg
            odom = Pose()
            odom.position.x = x
            odom.position.y = y
            
            qtuple = tfm.quaternion_from_euler(0, 0, theta)
            odom.orientation = Quaternion(qtuple[0], qtuple[1], qtuple[2], qtuple[3])
        except:
            # print out msg if there is an error parsing a serial msg
            print 'Cannot parse', splitData

def publish_vel():

    velPub = rospy.Publisher('/measured_vel', Twist, queue_size=1)
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
            
	    try:
            	# publish velocity as a Twist msg
            	vel = Twist()
            	vel.linear.x = (v_L+v_R)/2
		#print 'check 1'
            	if (abs(v_L) < abs(v_R)):
		    #print 'check 2 A'
                    turning_radius = abs((v_L+v_R)*b / (v_L-v_R))
                    vel.angular.z = vel.linear.x / turning_radius
		    #print 'check 2 B'
            	elif (abs(v_L) > abs(v_R)):
		    #print 'check 3 A'
                    turning_radius = -1.0 * abs(-1.0*(v_L+v_R)*b / (v_L-v_R))
                    vel.angular.z = vel.linear.x / turning_radius
		    #print 'check 3 B'
                else:
                    vel.angular.z = 0.0
		#print 'got here at least'	    
                velPub.publish(vel)
	    except:
		print 'ugh'
        except:
            print 'Cannot parse ', splitData
            

if __name__=='__main__':
    main()

