import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")

        ####################################
        # TODO
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0
        self.max_range = None

        self.resolution = None
        
        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)


    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.
        
        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A
        
        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        z = np.arange(self.table_width)
        z_s = np.arange(self.table_width)
        zz, zz_s = np.meshgrid(z, z_s)
        coords = np.dstack((zz_s,zz))

        a_hit = 0.74
        hits = np.apply_along_axis(self.p_hit, 2, coords)
        n_hits =  np.apply_along_axis(self.normalize, 0, hits)
        every = np.apply_along_axis(self.p, 2, coords)
        combined = a_hit*n_hits + every
        self.sensor_model_table = np.apply_along_axis(self.normalize, 0, combined)
                                    
    
    def p_hit(self,u):
        z = float(u[0])
        z_s = float(u[1])
        z_max = 200.0
        eta = 1.0
        sigma = self.sigma_hit
        if 0.0 <= z and z <= z_max:
            soln = eta*(1.0/np.sqrt(2.0*np.pi*sigma**2.0))*np.exp(-((z-z_s)**2.0)/(2.0*sigma**2.0))
            return soln
        return 0.0

    def p_short(self, z, z_s):
        if z_s == 0.0:
            return 0.0
        front = 2.0/z_s
        if 0.0 <= z and z <= z_s:
            soln = front*(1.0-z/z_s)
            return soln
        return 0.0
        
    def p_max(self, z, z_max):
        if z == z_max:
            return 1.0
        return 0.0


    def p_rand(self, z, z_max):
        if 0.0 <= z and z < z_max:
            return 1.0/z_max
        return 0.0

    def p(self,u):
        z_max = 200.0
        z = float(u[0])
        z_s = float( u[1])
        return self.alpha_short*self.p_short(z, z_s)+self.alpha_max*self.p_max(z, z_max)+self.alpha_rand*self.p_rand(z, z_max)


    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 
        z_max = 200
        scans = self.scan_sim.scan(particles)
<<<<<<< HEAD
        scalled_scans = np.round(scans/self.resolution)
        scalled_observations = np.round(observation/self.resolution)

        clipped_scans = np.where(scalled_scans > z_max, z_max, scalled_scans)
        clipped_observations = np.where(scalled_observations > z_max, z_max, scalled_observations)
        clipped_scans = np.where(scalled_scans < 0, 0, scalled_scans)
        clipped_observations = np.where(scalled_observations < 0, 0, scalled_observations)

        probabilities = np.apply_along_axis(self.get_probabilities, 1, clipped_scans, clipped_observations)
        return np.power(probabilities,1.0/2.2)
=======
        scalled_scans = np.round(scans*((self.table_width-1)/z_max))
        scalled_observations = np.round(observation*((self.table_width-1)/z_max))
        probabilities = np.apply_along_axis(self.get_probabilities, 1, scalled_scans,scalled_observations)
        normalized_probabilities = np.apply_along_axis(self.normalize,0,probabilities)
        return normalized_probabilities
>>>>>>> 8e847addc2fa7dc17129997f0825db4764c2083a
        ####################################

    def normalize(self,s):
        return s/sum(s)

    def get_probabilities(self, reading, actual):
        combined = np.dstack((reading,actual))
        soln = np.apply_along_axis(self.look_up_table,2,combined)
        return np.prod(soln[0])

    def look_up_table(self,location):
        z = int(location[0])
        z_s = int(location[1])
        return self.sensor_model_table[z_s][z]

    def map_callback(self, map_msg):
        # Set the resolution for the conversion:
        self.resolution = map_msg.info.resolution


        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
