#!/usr/bin/env python

import tf
from PIL import Image
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, TwistStamped, PoseWithCovarianceStamped, PoseArray, Quaternion
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

class ParticleFilter():
    '''
    This is a class used for localizing the mobile robot. It compares the seen
    point cloud to a known map and uses that to guess where the robot is likely
    to be, based on a Monte Carlo approach.
    '''

    def __init__(self, weighted_map, number_of_particles=50, hz=20, std_noise_x=0.01, std_noise_y=0.005):
        #-----CONSTANT VARIABLES-----#
        self.N = number_of_particles    # number of particles to simulate
        self.rate = rospy.Rate(hz)      # rate at which to try to publish position
        self.std_x = std_noise_x        # standard deviation of noise in x direction
        self.std_y = std_noise_y        # standard deviation of noise in y direction
	self.std_theta = 0.03
	self.m_to_px = 100		# pixels per meter
	self.dead = False        
        self.weighted_maps = np.tile(weighted_map, (self.N,1,1))
        self.map_width = weighted_map.shape[1]
        self.map_height = weighted_map.shape[2]
        #-----CALLBACK VARIABLES-----#
        self.latest_cloud = None
        self.latest_odom = None
        self.last_update_time = None
        
        #-----PARTICLES-----#
        self.particles = np.zeros((self.N,3,1))
	self.weights = np.zeros(self.N)
    	start = PoseWithCovarianceStamped()
    	start.pose.pose.position.x = 2.0
    	start.pose.pose.position.y = 0.3
    	start.pose.pose.orientation.z = 1.0
    	start.pose.pose.orientation.w = 1.0
    	self.setLocation(start)

        #-----COMMUNICATION PARAMETERS-----#
        # odometry should be a twist message
        self.odom_topic = '/measured_vel'
        # cloud should be a pointcloud2 message
        self.cloud_topic = '/sliced_cloud'
        # this topic will be updated with the inferred pose of the robot
        self.ref_frame = '/robot_base'
  
        # initialize publishers/subscribers
	self.part_pub = rospy.Publisher('/particles', PoseArray, queue_size=1)
        self.frame_pub = rospy.Publisher('/robot_base', PoseStamped, queue_size=1)
        self.vel_sub = rospy.Subscriber('/measured_vel', TwistStamped, self.velCallback)
        self.cloud_sub = rospy.Subscriber(self.cloud_topic, Marker, self.cloudCallback)
	self.reset_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.setLocation)
	self.dead_sub = rospy.Subscriber('/dead', Bool, self.deadCallback)

        #-----RUN THE FILTER-----#
        while not rospy.is_shutdown():
	    # show the particles
	    self.publishParticles()
	    self.runFilter()
 	    #print self.weights
            self.rate.sleep()

    def deadCallback(self, msg):
	self.dead = msg.data

    def publishParticles(self):
        poseAr = PoseArray()
	poseAr.header.frame_id = '/map'
	for p in np.copy(self.particles):
	    pose = Pose()
            pose.position.x = p[0]
	    pose.position.y = p[1]
	    quat = tf.transformations.quaternion_from_euler(0,0,p[2])
            pose.orientation.x = quat[0]
	    pose.orientation.y = quat[1]
	    pose.orientation.z = quat[2]
	    pose.orientation.w = quat[3]
	    poseAr.poses.append(pose)
	self.part_pub.publish(poseAr)
            
    def velCallback(self, twistSt):
        '''
        Updates the particlGe filter with the new odometry data

        msg     TwistStamped message
        output  Updates self.latest_odom with [linear_velocity, angular_velocity]
        '''
	# set the current time on the first itteration
        if self.last_update_time == None:
            self.last_update_time = rospy.get_time()
            return

        vel = np.array([twistSt.twist.linear.x, twistSt.twist.angular.z])
        
	# use vel to find change in position from particle frames
        dt = rospy.get_time() - self.last_update_time
        self.last_update_time = rospy.get_time()
	if vel[1] == 0:
	    # driving straight
	    dx = dt*vel[0]
	    dy = 0
	    dtheta = 0.0
	else:
            r = vel[0] / vel[1]
            dtheta = vel[1]*dt
            dx = r*np.sin(dtheta)
            dy = r*(1-np.cos(dtheta))

        # add noise to the motion
	dt = np.zeros((self.N,1,1))
        if self.std_x != 0.0:
 	    dx = dx + np.random.normal(0.0, self.std_x, (self.N,1,1))
        else:
	    dx = dx + np.zeros((self.N,1,1))
	if self.std_y != 0.0:
	    dy = dy + np.random.normal(0.0, self.std_x, (self.N,1,1))
	else:
	    dy = dy + np.zeros((self.N,1,1))
	if self.std_theta != 0.0:
	    dt = np.random.normal(0.0, self.std_theta, (self.N,1,1))
	else:
	    dt = np.zeros((self.N,1,1))
        # convert to change in global pose
        delta_x = dx*np.cos(self.particles[:,2:3]) - dy*np.sin(self.particles[:,2:3])
        delta_y = dx*np.sin(self.particles[:,2:3]) + dy*np.cos(self.particles[:,2:3])
        delta_theta = (dtheta + dt)
	#print 'dhteta: ', dtheta
	#print 'dt: ', min(dt), max(dt)
	self.particles[:,0:1] += delta_x
        self.particles[:,1:2] += delta_y
	self.particles[:,2:3] += delta_theta


    def cloudCallback(self, msg):
        '''
        Updates the weight of each particle based on the cloud data

        msg     a marker message
        output  Updates self.particle_weights with probability of each particle being the true location
        '''
	#print 'good map'
	#print self.weighted_maps[(self.weighted_maps!=0.0)*(self.weighted_maps!=1.0)]
	points = [[],[]]
        for p in msg.points:
	    points[0].append(p.x)
	    points[1].append(p.y)
	#print 'Ack'
	#print points
	N_points = len(points[0])
        pts = np.zeros((1,3,len(points[0])))
        pts[0,1:3,:] = points
 
        # repeat for each particle
        pts = np.tile(pts, (self.N, 1,1))
        pts[:,0:1,:] = np.tile(np.arange(self.N).reshape((self.N,1,1)), (1,1,pts.shape[2]))
        
        # convert from particle frames to global frame
        points = np.zeros(pts.shape)
        points[:,0:1,:] = pts[:,0:1,:]
        points[:,1:2] = pts[0,1:2]*np.cos(self.particles[:,2:3]) - pts[0,2:3]*np.sin(self.particles[:,2:3]) + self.particles[:,0:1]
        points[:,2:3] = pts[0,1:2]*np.sin(self.particles[:,2:3]) + pts[0,2:3]*np.cos(self.particles[:,2:3]) + self.particles[:,1:2]
	# check transformation
	if False:
	    pts = np.copy(points[0,1:3,:])
	    print pts.shape
	    marker = Marker()
	    marker.scale.x = 0.01
	    marker.scale.y = 0.01
	    marker.scale.z = 0.01
	    marker.color.g = 1.0
	    marker.color.a = 1.0
	    marker.type = 7
	    marker.header.frame_id = '/map'
	    pub = rospy.Publisher('/testing', Marker, queue_size=1)
	    for a in range(pts.shape[1]):
		p = Point()
		p.x = pts[0,a]
		p.y = pts[1,a]
		marker.points.append(p)
    	    pub.publish(marker)
	# convert distance coordinates to pixel indicies
	points[:,1:2] = points[:,1:2]*self.m_to_px
        points[:,2:3] = points[:,2:3]*self.m_to_px
        points = points.astype(int)
	
        # reshape to 2d for indexing into weighted map
        points = np.reshape(points.transpose(2,0,1), (points.size//3,3), order='F')
        # remove points outside of the map boundaries
	points = points[((points[:,1:2] >=0) * (points[:,1:2] < self.map_width)).flatten(), :]
        points = points[((points[:,2:3] >=0) * (points[:,2:3] < self.map_height)).flatten(), :] 
	probs = self.weighted_maps[points[:,0], points[:,1], points[:,2]].astype(float)
	#print 'map sample: ', self.weighted_maps[points[25,0], points[25,1], points[25,2]].astype(float)
	#print 'Points: ', points.shape
	#print 'Probs:  ', probs.shape
	#print 'Good probs: '
	#print probs[(probs != 1.0)*(probs != 0.0)]
	#probs = np.random.uniform(0.0, 1.0, points.shape[0])
	#probs += 0.01
	groups = np.arange(self.N)
	prob_sum = []
	counts = []
	for i in range(self.N):
	    counts.append(np.sum(i==points[:,0]))
	max_counts = max(counts)
	for g in range(self.N):
	    #prob_sum.append((probs[g==points[:,0]]).size)
	    prob_sum.append(np.sum(probs[g==points[:,0]]))
	    #prob_sum.append(np.prod(probs[g==points[:,0]]+0.1)*0.01**(max_counts-counts[g]))
	#print prob_sum
	prob_sum = np.array(prob_sum)
	#print 'prob sum: ', prob_sum	
	#prob_sum =  np.sum(self.weighted_maps[points[:,0], points[:,1], points[:,2]].reshape((self.N, points.shape[0]//self.N)), axis=1)
	if N_points < 100 or self.dead:
	    self.std_x = 0.0 # 0.0000001
	    self.std_y = 0.0 #000001
	    self.std_theta = 0.0 #000001
	    return
	else:
	    self.std_x = 0.01
	    self.std_y = 0.005
	    self.std_theta = 0.003
	weights = prob_sum / np.sum(prob_sum)
	# resample the particles based on the new weights
        particle_ind = np.arange(self.N)
        keep_ind = np.random.choice(particle_ind, self.N, p=weights)
        self.particles = self.particles[keep_ind, :]
	new_weights = weights[keep_ind]
	self.weights = new_weights / np.sum(new_weights)

    def runFilter(self):
        '''
        Updates the particle positions and uses the current weights to publish an inferred pose
        '''
        # get the inferred position
	pos = self.inferLocation()
	poseSt = PoseStamped()
	poseSt.header.frame_id = '/map'
	poseSt.pose.position.x = pos[0]
	poseSt.pose.position.y = pos[1]
	poseSt.pose.orientation.z = pos[2]
        # publish the inferred position
	self.frame_pub.publish(poseSt)


    def setLocation(self, msg):
	pos = []
	pos.append(msg.pose.pose.position.x)
	pos.append(msg.pose.pose.position.y)
	orien = msg.pose.pose.orientation
	quat = [orien.x, orien.y, orien.z, orien.w]
	euler = tf.transformations.euler_from_quaternion(quat)
	pos.append(euler[2])
	self.particles[:,0:1] = np.random.normal(pos[0], self.std_x, size=(self.N,1,1))
	self.particles[:,1:2] = np.random.normal(pos[1], self.std_y, size=(self.N,1,1))
	self.particles[:,2:3] = np.random.normal(pos[2], self.std_theta*4.0, size=(self.N,1,1))
        

    def inferLocation(self):
	#print self.weights
	#pt = np.copy(self.particles[np.argmax(self.weights),:])
	#return [pt[0], pt[1], pt[2]]
	x_hist, edges = np.histogram(self.particles[:,0,0], weights=self.weights)
        x_inf = (edges[np.argmax(x_hist)] + edges[np.argmax(x_hist)]) / 2.0
        y_hist, edges = np.histogram(self.particles[:,1,0], weights=self.weights)
        y_inf = (edges[np.argmax(y_hist)] + edges[np.argmax(y_hist)]) / 2.0
        t_hist, edges = np.histogram(self.particles[:,2,0], weights=self.weights)
        t_inf = (edges[np.argmax(t_hist)] + edges[np.argmax(t_hist)]) / 2.0
        return [x_inf, y_inf, t_inf]



if __name__ == '__main__':
    rospy.init_node('particle_filter')
    weight_map = np.array(Image.open("/home/robot/mobile_robot_ws/src/mobile_robot/maps/212_weight_map_with_delta.png"))[:,:,0].astype(float)
    weight_map = weight_map / np.max(weight_map)
    print weight_map[weight_map != 0]
    weight_map = np.flipud(weight_map)
    # make map 1xwidthxheight
    #weights = np.transpose(weight_map)
    weights = np.transpose(weight_map).reshape((1,weight_map.shape[1],weight_map.shape[0]))
    # flip logic so black is high
    weights = 1 - weights
    # look at the weight map to see if it seems correct
    if False: #while True:
    	marker = Marker()
    	marker.header.frame_id = '/map'
	marker.scale.x = 0.1
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	marker.color.g = 1.0
	marker.color.a = 1.0
	marker.type = 7
	w = np.copy(weights)
	N = 0
    	for a in range(w.shape[0]):
	    for b in range(w.shape[1]):
	    	for c in range(w.shape[2]):
		    if w[a,b,c]==0:
			continue
		    p = Point()
		    p.x = b / 100.0
		    p.y = c / 100.0
		    marker.points.append(p)
		    print a,b,c,' : ',p.x, p.y
        pub = rospy.Publisher('/weights', Marker, queue_size=1)
        pub.publish(marker)
    
    pf = ParticleFilter(weights)
    rospy.spin()
