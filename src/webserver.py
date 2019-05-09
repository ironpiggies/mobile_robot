#!/usr/bin/env python
#author achuwils
'''
set up a wifi hotspot in mobile robot and connect the delta robot PC to it
check whether the IP address of the mobile robot is 10.0.42.1 ( by default)
if, not, change the value in the tcp_commander.py file in delta robot

The wifi hotspot can be set up by running the following command in mobile robot

nmcli device wifi hotspot con-name ironpiggy ssid ironpiggy band bg password beeper3245

where wifi name is ironpiggy and password is beeper3245



'''
import roslib
import rospy
import sys
from thread import *
import socket
from std_msgs.msg import String
from threading import Thread

HOST = ''
PORT=8888

rospy.set_param('status', '0')

BUFFER_SIZE =50
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#print "socket created"
try:
    s.bind((HOST,PORT))
except socket.error as msg:
    print 'Bind Failed,Error COde : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

#print "socket bind complete"
s.listen(10)
#print "socket now listening"


    

def clientthread(conn):
    pub = rospy.Publisher('tcp_command', String,queue_size = 5)
    while True:
        try:                              #TO KEEP THE SERVER FROM TIMING OUT
            data = conn.recv(BUFFER_SIZE)
            if(len(data)>0):
                pub.publish(data)
                robo_status=rospy.get_param("/status")
                conn.send(robo_status)

        except socket.timeout:
            print time.time()
            data = ''
            continue                      
        if not data: break
        
    conn.close()    

def server():
    r = rospy.Rate(300)
    while not rospy.is_shutdown():
        conn, addr = s.accept()
        conn.settimeout(300) #setting timeout for connection
        #print ' Connected with ' addr[0] + ':' + str(addr[1])
        start_new_thread(clientthread,(conn,))
    s.close()
    
def main():
    rospy.init_node('piggy_tcp_interface')
    server()
    rospy.spin()


if __name__ == '__main__': 
	main()
