#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import Point
import ros_numpy
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
		self.up_to_date = True
		# convert point cloud into numpy array in robot frame
		points = cloudToArray(self.last_cloud)
		# remove points outside of the expected z range
		points = points[np.logical_and(points[:,2]>self.z_min, points[:,2]<self.z_max), 0:2]
                # convert to pixel locations
		points = (points * self.m_to_px).astype(int)
		# remove duplicates
		rand = np.random.rand(points.shape[1])
		y = points.dot(rand)
		unique, index = np.unique(y, return_index=True)
		points = points[index].astype(float)
		# convert back to meters
		points = points / self.m_to_px
		# convert to Marker
		marker = arrayToSphereList(points)
		# publish
		self.pub.publish(marker)
    
    def cloudCallback(self, cloud):
        self.last_cloud = cloud
        self.up_to_date = False



def cloudToArray(cloud):
    '''
    Converts a PointCloud2 message to a numpy array [[x,y,z],...]
    '''
    #pts = []
    #for pt in pc2.read_points(cloud, skip_nans=True):
    #    pts.append((pt[0], pt[1], pt[2]))
    #pts2 = np.array(cloud.data)
    #print pts2.shape
    #return pts2
    #return np.array(pts)
    #return ros_numpy.point_cloud2.get_xyz_points(cloud)
    x_offset = -0.24
    y_offset =  0.00
    z_offset = -0.35

    pc = ros_numpy.numpify(cloud)
    points = np.zeros((pc.size,3))
    points[:,0]=pc['z'].reshape(pc.size) + x_offset
    points[:,1]=pc['x'].reshape(pc.size) + y_offset
    points[:,2]=pc['y'].reshape(pc.size) + z_offset
    return points

def arrayToSphereList(array):
    print 'Number of points: ', array.shape[0]
    marker = Marker()
    marker.type = 8
    marker.color.r = 1.0
    marker.color.a = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = 0.1,0.1,0.1
    marker.header.frame_id = '/base_link'
    for pt in array:
	p = Point()
	p.x, p.y = pt[0], pt[1]
	marker.points.append(p)
    return marker

if __name__ == '__main__':
    rospy.init_node('cloud_slicer')
    cs = CloudSlicer()
    rospy.spin()
