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

    def __init__(self, weighted_maps, number_of_particles=10, hz=50, std_noise_x=0.1, std_noise_y=0.05):
        
        #-----CONSTANT VARIABLES-----#
        self.N = number_of_particles    # number of particles to simulate
        self.rate = rospy.Rate(hz)      # rate at which to try to publish position
        self.std_x = std_noise_x        # standard deviation of noise in x direction
        self.std_y = std_noise_y        # standard deviation of noise in y direction
        
        self.weighted_top = weighted_maps[:,:,0]
        self.weighted_bot = weighted_maps[:,:,1]
        
        #-----CALLBACK VARIABLES-----#
        self.latest_cloud = None
        self.latest_odom = None
        self.last_update = None
        
        #-----PARTICLES-----#
        self.particles = np.zeros((N,3))

        #-----COMMUNICATION PARAMETERS-----#
        # odometry should be a twist message
        self.odom_topic = '/odom'
        # cloud should be a pointcloud2 message
        self.cloud_topic = '/cloud_data'
        # this topic will be updated with the inferred pose of the robot
        self.ref_frame = '/base_link'
  
        # initialize publishers/subscribers
        self.frame_pub = rospy.Publisher(self.ref_frame, PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, TwistStamped, odomCallback)
        self.cloud_sub = rospy.Subscriber(self.cloud_topic, PointCloud2, cloudCallback)

        #-----RUN THE FILTER-----#
        while not rospy.is_shutdown():
            self.runFilter()
            self.rate.sleep()

            
    def odomCallback(self, msg):
        '''
        Updates the particle filter with the new odometry data

        msg     TwistStamped message
        output  Updates self.latest_odom with [linear_velocity, angular_velocity]
        '''
        self.latest_odom = np.array([msg.twist.linear.x, msg.twist.angular.z])


    def cloudCallback(self, msg):
        '''
        Updates the weight of each particle based on the cloud data

        msg     pointCloud2 message
        output  Updates self.particle_weights with probability of each particle being the true location
        '''
        height_maps = np.zeros((self.N, map_width, map_height))

        # plot each point from the cloud onto the height_maps
        for point in pc2.read_points(msg, skip_nans=True):

            # convert from particle frames to global frame
            x = point[0]*np.cos(self.particles[:,2:3]) - point[1]*np.sin(self.particles[:,2:3]) + particles[:,0:1]
            y = point[0]*np.sin(self.particles[:,2:3]) + point[1]*np.cos(self.particles[:,2:3]) + particles[:,1:2]
            coords = np.hstack((np.arange(self.N).reshape((self.N,1)), x, y))
           
            # convert distanct coordinates to pixel indicies
            ind = (coords*self.map_scale).astype(int)
            
            # remove indicies where x,y, or z are out of bounds
            ind = ind[np.logical_and(ind[:,1:2] >= 0, ind[:,1:2] < self.map_width), :]
            ind = ind[np.logical_and(ind[:,2:3] >= 0, ind[:,2:3] < self.map_height), :]
            ind = ind[np.logical_and(ind[:,2:3] >= self.z_min, ind[:,2:3] < z_max), :]

            # plot the height on the height_maps, keeping the highest where repeats occurr
            height_maps[ind] = np.max(np.array([height_maps[ind], np.repeat(point[2], ind.shape[0])]), axis=1)

        # break the height_maps into top_maps and bottom_maps
        top_maps = np.where(height_maps >= self.z_upper_thresh, height_maps*0+1, 0)
        bot_maps = np.where(height_maps >= self.z_lower_thresh, height_maps*0+1, 0)

        # sum each map, and then sum each corresponding map together
        top_weighted_sums = np.sum(top_maps*self.weighted_top, axis=0)
        top_weighted_sums = top_weighted_sums / np.max(top_weighted_sums)
        bot_weighted_sums = np.sum(bot_maps*self.weighted_bot, axis=0)
        bot_weighted_sums = bot_weighted_sums / np.max(bot_weighted_sums)

        # weight each particle relative to the total sum of its maps
        self.weights = 0.5*top_weighted_sums + 0.5*bot_weighted_sums

        # resample the particles based on the new weights
        particle_ind = np.arange(self.N)
        keep_ind = np.random.choice(particle_ind, p=self.weights)
        self.particles = self.particles[keep_ind, :]


    def runFilter(self):
        '''
        Updates the particle positions and uses the current weights to publish an inferred pose
        '''

        # use odom to find change in position from particle frames
        dt = rospy.get_time() - self.last_update_time
        self.last_update_time = rospy.get_time()
        r = self.odom[0] / self.odom[1]
        dtheta = self.odom[1]*dt
        dx = r*(np.cos(dtheta)-1)
        dy = r*np.sin(dtheta)

        # convert to change in global pose
        delta_x = dx*np.cos(self.particles[:,2:3]) - dy*np.sin(self.particles[:,2:3])
        delta_y = dx*np.sin(self.particles[:,2:3]) + dy*np.cos(self.particles[:,2:3])
        self.particles[:,0:1] += delta_x
        self.particles[:,1:2] += delta_y

        # get the inferred position
        # publish the inferred position


    def setParticleLocations(self):
        pass

    def inferLocation(self):
        x_hist = np.histogram(self.particles[:,0:1], weights=self.weights)
        x_inf = np.min(self.particles[:,0]) + (np.argmax(x_hist)+0.5) * (np.max(self.particles[:,0])-np.min(self.particles[:,0]))
        y_hist = np.histogram(self.particles[:,1:2], weights=self.weights)
        y_inf = np.min(self.particles[:,1]) + (np.argmax(y_hist)+0.5) * (np.max(self.particles[:,1])-np.min(self.particles[:,1]))
        t_hist = np.histogram(self.particles[:,0:3], weights=self.weights)
        t_inf = np.min(self.particles[:,3]) + (np.argmax(t_hist)+0.5) * (np.max(self.particles[:,2])-np.min(self.particles[:,2]))
        return [x_inf, y_inf, t_inf]

