#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import Point
# takes in the camera topic and finds points within a specified height
# also does the transformation from camera frame to robot_base
# publishes points projected onto a 2d plane in the form of a SphereList


class CloudSlicer():
    
    def __init__(self):
        self.z_min = 0.02
        self.z_max = 0.60
	self.m_to_px = 1000

        self.pub = rospy.Publisher('/sliced_cloud', Marker, queue_size=1)
        self.sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.cloudCallback)
    
        self.last_cloud = None
        self.up_to_date = True

        while not rospy.is_shutdown():
            if not self.up_to_date:
		# convert point cloud into numpy array
                t_start = rospy.get_time()
		t = rospy.get_time()
		camera_points = cloudToArray(self.last_cloud)
		print 'Section 0: ', rospy.get_time() - t
	
                t = rospy.get_time()
		# switch to reference frame of robot_base
                points = transformCameraToRobotBase(camera_points)
		print 'Section 1: ',rospy.get_time() - t
                t = rospy.get_time()
		# remove points outside of the expected z range
		points = points[np.logical_and(points[:,2]>self.z_min, points[:,2]<self.z_max), 0:2]
		print 'Section 2: ',rospy.get_time() - t
                t = rospy.get_time()
                # convert to pixel locations
		points = (points * self.m_to_px).astype(int)
		print 'Section 3: ',rospy.get_time() - t
                t = rospy.get_time()
		# remove duplicates
		rand = np.random.rand(points.shape[1])
		y = points.dot(rand)
		unique, index = np.unique(y, return_index=True)
		points = points[index]
		print 'Section 4: ',rospy.get_time() - t
                t = rospy.get_time()
		# convert back to meters
		points = points / self.m_to_px
		print 'Section 5: ',rospy.get_time() - t
                t = rospy.get_time()
		# convert to Marker
		marker = arrayToSphereList(points)
		print 'Section 6: ',rospy.get_time() - t
                t = rospy.get_time()
		# publish
		self.pub.publish(marker)
		print 'Section 7: ', rospy.get_time() -t
		print 'Total time: ', rospy.get_time() - t_start
	    else:
		print 'Kewl'
    def cloudCallback(self, cloud):
        self.last_cloud = cloud
        self.up_to_date = False


def transformCameraToRobotBase(points):
    '''
    Converts a series of points from the camera frame to robot_base
    '''
    x_offset = -0.24
    y_offset =  0.00
    z_offset = -0.35

    # robot x = camera z
    x = points[:,2:3] + x_offset
    # robot y = camera x
    y = points[:,0:1] + y_offset
    # robot z = camera y
    z = points[:,1:2] + z_offset

    return np.hstack((x,y,z))

def cloudToArray(cloud):
    '''
    Converts a PointCloud2 message to a numpy array [[x,y,z],...]
    '''
    pts = []
    for pt in pc2.read_points(cloud, skip_nans=True):
        pts.append((pt[0], pt[1], pt[2]))
    return np.array(pts)

def arrayToSphereList(array):
    marker = Marker()
    marker.type = 8
    marker.color.r = 1.0
    marker.color.a = 1.0
    marker.header.frame_id = '/robot_base'
    N = 0
    for pt in array:
	p = Point()
	p.x, p.y = pt[0], pt[1]
	marker.points.append(p)
	N += 1
    print 'number of points: ', N
    return marker

if __name__ == '__main__':
    rospy.init_node('cloud_slicer')
    cs = CloudSlicer()
    rospy.spin()
