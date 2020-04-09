import numpy as np
import rospy

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        self.DETERMINISTIC = rospy.get_param('deterministic', True)
        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        
        ####################################
        odometry = np.array(odometry)
        particles = np.array(particles)
        updated =  np.apply_along_axis(self.transform, 1, particles, odometry)
        return updated

    def transform(self, particle, odometry):
        """
        Computes the new pose of a particle given an odometry vector
        """
        if not self.DETERMINISTIC:
            noise = np.array([np.random.normal(0,abs(eta*particle[0])), np.random.normal(0,abs(eta*particle[1])),np.random.normal(0,abs(eta*particle[2]))])
            particle = particle + noise
        rot_matrix = np.array([[np.cos(particle[2]),-np.sin(particle[2])],[np.sin(particle[2]), np.cos(particle[2])]])
        relative_loc = np.array([[odometry[0]],[odometry[1]]])
        loc = np.dot(rot_matrix,relative_loc)
        return np.array([loc[0][0]+particle[0], loc[1][0]+particle[1], particle[2]+odometry[2]])

    
#     def add_noise(self, particle, eta):
#         """
#         Add some random noise with noise-factor eta to the particles
#         """
#         x = particle[0] + np.random.normal(0,abs(eta*particle[0]))
#         y = particle[1] + np.random.normal(0,abs(eta*particle[1]))
#         th = particle[2] + np.random.normal(0,abs(eta*particle[2]))
#         return [x,y,th]

