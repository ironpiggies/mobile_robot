#!/usr/bin/env python

# takes in the camera topic and finds points within a specified height
# also does the transformation from camera frame to robot_base
# publishes points projected onto a 2d plane in the form of a SphereList


class CloudSlicer():
    
    def __init__(self):
        self.z_min = 0.02
        self.z_max = 0.60

        self.pub = rospy.Publisher('/sliced_cloud', SphereList, queue_size=1)
        self.sub = rospy.Subscriber('/camera_cloud_topic', PointCloud2, cloudCallback)
    
        self.last_cloud = None
        self.up_to_date = True

        while not rospy.is_shutdown():
            if not self.up_to_date:
                camera_points = cloudToArray(self.last_cloud)
                points = tranfromCameraToRobotBase(camera_points)
                points = robot_points[np.logical_and(points[:,2:3]>self.z_min, points[:,2:3]<self.z_max), :]
                self.pub.publish(arrayToShereList(points))

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
    # robot y = -camera x
    y = -1*points[:,0:1] + y_offset
    # robot z = -camera y
    z = -1*points[:,1:2] + z_offset

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
    pass
