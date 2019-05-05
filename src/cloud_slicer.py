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
	self.m_to_px = 100
	
	self.robot_theta = 0.0
	self.robot_x = 0.0
	self.robot_y = 0.0

        self.center_field = 1.21
        self.field_buffer = 0.1
        self.pub = rospy.Publisher('/sliced_cloud', Marker, queue_size=1)
        self.sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.cloudCallback)
    
        self.last_cloud = None
        self.up_to_date = True

        while not rospy.is_shutdown():
            if not self.up_to_date:
		self.up_to_date = True
		t = rospy.get_time()
		# convert point cloud into numpy array in robot frame
		points = self.extractFromCloud(self.last_cloud)

		# convert to Marker
		marker = self.arrayToSphereList(points)
		
		# publish
		self.pub.publish(marker)
    
    def cloudCallback(self, cloud):
        self.last_cloud = cloud
        self.up_to_date = False

    def extractFromCloud(self, cloud):
        '''
        Converts a PointCloud2 message to a numpy array [[x,y,z],...]
        '''
        x_offset = -0.24
        y_offset =  0.00
        z_offset = 0.35
        # read the point cloud data
        pc = ros_numpy.numpify(cloud)
        points = np.zeros((pc.size,3))
        points[:,0]=pc['z'].reshape(pc.size)
        points[:,1]=-1.0*pc['x'].reshape(pc.size)
        points[:,2]=-1.0*pc['y'].reshape(pc.size)
        # handle rotation of y axis
        theta_y = 0.02
        points[:,0] -= points[:,2]*0.02
        points[:,2] += points[:,0]*0.02

        # handle offsets
        points[:,0] += x_offset
        points[:,1] += y_offset
        points[:,2] += z_offset
	
        # remove points outside of the expected z range
        points = points[np.logical_and(
                     points[:,2]>self.z_min, 
                     points[:,2]<self.z_max), 0:2]
    
        # convert to relative pixel locations
        points = (points * self.m_to_px).astype(int)
    
        # remove duplicates
        x = np.random.rand(points.shape[1])
        y = points.dot(x)
        unique, index = np.unique(y, return_index=True)
        points = points[index].astype(float)
        # convert back into meters
        points = points / self.m_to_px
        
        # look at the points from the reference frame of the map
        map_x = points[:,0]*np.cos(self.robot_theta) - points[:,1]*np.sin(self.robot_theta) + self.robot_x
        map_y = points[:,0]*np.sin(self.robot_theta) + points[:,1]*np.cos(self.robot_theta) + self.robot_y
    
        # only keep points likely to be within the field
        points = points[np.logical_and(
		     np.abs(map_x - self.center_field)<self.center_field+self.field_buffer,
		     np.abs(map_y - self.center_field)<self.center_field+self.field_buffer), :]
        return points

    def arrayToSphereList(self, array):
        marker = Marker()
        marker.type = 7
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = 0.01,0.01,0.01
        marker.header.frame_id = '/robot_base'
        for pt in array:
	    p = Point()
 	    p.x, p.y = pt[0], pt[1]
	    if pt.size == 3:
	        p.z = pt[2]
	    marker.points.append(p)
        return marker

if __name__ == '__main__':
    rospy.init_node('cloud_slicer')
    cs = CloudSlicer()
    rospy.spin()
