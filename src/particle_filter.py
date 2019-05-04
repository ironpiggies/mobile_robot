import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class ParticleFilter():
    '''
    This is a class used for localizing the mobile robot. It compares the seen
    point cloud to a known map and uses that to guess where the robot is likely
    to be, based on a Monte Carlo approach.
    '''

    def __init__(self, weighted_map, number_of_particles=100, hz=50, std_noise_x=0.1, std_noise_y=0.05):
        
        #-----CONSTANT VARIABLES-----#
        self.N = number_of_particles    # number of particles to simulate
        self.rate = rospy.Rate(hz)      # rate at which to try to publish position
        self.std_x = std_noise_x        # standard deviation of noise in x direction
        self.std_y = std_noise_y        # standard deviation of noise in y direction
	self.m_to_px = 100		# pixels per meter
        
        self.weighted_maps = np.tile(weighted_map, (self.N,1,1))
        self.map_width = weighted_map.shape[1]
        self.map_height = weighted_map.shape[2]
        #-----CALLBACK VARIABLES-----#
        self.latest_cloud = None
        self.latest_odom = None
        self.last_update_time = None
        
        #-----PARTICLES-----#
        self.particles = np.zeros((self.N,3,1))
	self.weights = np.zeros((self.N,1,1))

        #-----COMMUNICATION PARAMETERS-----#
        # odometry should be a twist message
        self.odom_topic = '/odom'
        # cloud should be a pointcloud2 message
        self.cloud_topic = '/sliced_cloud'
        # this topic will be updated with the inferred pose of the robot
        self.ref_frame = '/base_link'
  
        # initialize publishers/subscribers
        self.frame_pub = rospy.Publisher(self.ref_frame, PoseStamped, queue_size=1)
        self.vel_sub = rospy.Subscriber(self.odom_topic, TwistStamped, self.velCallback)
        self.cloud_sub = rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloudCallback)

        #-----RUN THE FILTER-----#
        while not rospy.is_shutdown():
            self.runFilter()
            self.rate.sleep()
            break

            
    def velCallback(self, twistSt):
        '''
        Updates the particle filter with the new odometry data

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
	    dx = dt*odom[0]
	    dy = 0
	else:
            r = self.odom[0] / self.odom[1]
            dtheta = self.odom[1]*dt
            dx = r*np.sin(dtheta)
            dy = r*(1-np.cos(dtheta))

        # add noise to the motion
        dx += np.random.normal(0, self.std_x, (self.N,1))
        dy += np.random.normal(0, self.std_y, (self.N,1))

        # convert to change in global pose
        delta_x = dx*np.cos(self.particles[:,2:3]) - dy*np.sin(self.particles[:,2:3])
        delta_y = dx*np.sin(self.particles[:,2:3]) + dy*np.cos(self.particles[:,2:3])
        self.particles[:,0:1] += delta_x
        self.particles[:,1:2] += delta_y


    def cloudCallback(self, msg):
        '''
        Updates the weight of each particle based on the cloud data

        msg     a marker message
        output  Updates self.particle_weights with probability of each particle being the true location
        '''
 	points = [[],[]]
        for p in msg.points:
	    points[0].append(p.x)
	    points[1].append(p.y)
	
        pts = np.zeros((1,3,len(points[0])))
        pts[0,0:2,:] = points
        
        # repeat for each particle
        pts = np.tile(pts, (self.N, 1,1))
        pts[:,0:1,:] = np.tile(np.arange(self.N).reshape((self.N,1,1)), (1,1,pts.shape[2]))
        

        # convert from particle frames to global frame
        points = np.zeros(pts.shape)
        points[:,0:1,:] = pts[:,0:1,:]
        points[:,1:2] = pts[0,1:2]*np.cos(self.particles[:,2:3]) - pts[0,2:3]*np.sin(self.particles[:,2:3]) + self.particles[:,0:1]
        points[:,1:2] = pts[0,1:2]*np.sin(self.particles[:,2:3]) + pts[0,2:3]*np.cos(self.particles[:,2:3]) + self.particles[:,1:2]

        # convert distance coordinates to pixel indicies
        points[:,1:2] = points[:,1:2]*self.m_to_px
        points[:,2:3] = points[:,2:3]*self.m_to_px
        points = points.astype(int)

        # reshape to 2d for indexing into weighted map
        points = np.reshape(points.transpose(2,0,1), (points.size//3,3), order='F')
        
        # remove points outside of the map boundaries
        points = points[((points[:,1:2] >=0) * (points[:,1:2] < self.map_width)).flatten(), :]
        points = points[((points[:,2:3] >=0) * (points[:,2:3] < self.map_height)).flatten(), :] 

        prob_sum =  np.sum(self.weighted_maps[points[:,0], points[:,1], points[:,2]].reshape((self.N, points.shape[0]//self.N)), axis=1)
        weights = prob_sum / np.sum(prob_sum)
	
        # resample the particles based on the new weights
        particle_ind = np.arange(self.N)
        keep_ind = np.random.choice(particle_ind, self.N, p=weights)
        self.particles = self.particles[keep_ind, :]
	new_weights = weights[keep_ind].reshape(self.N,1)
	self.weights = new_weights / np.sum(new_weights)

    def runFilter(self):
        '''
        Updates the particle positions and uses the current weights to publish an inferred pose
        '''
        # get the inferred position
	pos = self.inferLocation()
	poseSt = PoseStamped()
	poseSt.header.frame_id = self.ref_frame
	poseSt.pose.position.x = pos[0]
	poseSt.pose.position.y = pos[1]
	poseSt.pose.orientation.z = pos[2]
        # publish the inferred position
	self.frame_pub.publish(poseSt)


    def setParticleLocations(self):
        pass

    def inferLocation(self):
        x_hist, edges = np.histogram(self.particles[:,0:1], weights=self.weights)
        x_inf = (edges[np.argmax(x_hist)] + edges[np.argmax(x_hist)]) / 2.0
        y_hist, edges = np.histogram(self.particles[:,1:2], weights=self.weights)
        y_inf = (edges[np.argmax(y_hist)] + edges[np.argmax(y_hist)]) / 2.0
        t_hist, edges = np.histogram(self.particles[:,2:3], weights=self.weights)
        t_inf = (edges[np.argmax(t_hist)] + edges[np.argmax(t_hist)]) / 2.0
        return [x_inf, y_inf, t_inf]



if __name__ == '__main__':
    rospy.init_node('testing')
    weight_map = 1+np.zeros((1,244,244))
    pf = ParticleFilter(weight_map)
    x_vals = [0.0, 0.5, 1.0]*1000
    y_vals = [0.1, 0.2, 0.3]*1000
    points = [x_vals, y_vals]
    t_start = rospy.get_time()
    pf.cloudCallback(points)
    t_end = rospy.get_time()
    print 'Time: ', t_end-t_start
