#!/usr/bin/env python2

import rospy
import threading
import numpy as np
from sensor_model import SensorModel
from motion_model import MotionModel
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseArray, Pose
from visualization_msgs.msg import Marker


class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.lidarCallback, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odomCallback, # TODO: Fill this in
                                          queue_size=1)
        self.N = 200
        self.lock = threading.RLock()


        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.initializationCallback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        self.odom_msg = Odometry()

        self.viz_pub = rospy.Publisher("viz_marker", PoseArray, queue_size = 10)
        self.viz_msg = PoseArray()

        self.br = TransformBroadcaster()

        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

    def initializationCallback(self, msg):
        # return
        #access the position and orientation data from the clicked point
        particle_1 = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w]
        orientation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        #get theta value from the orientation
        particle_1[2] = euler_from_quaternion(orientation)[2]
        #add noise using motion_model noise function
        with self.lock:
            self.particles = np.array([self.motion_model.add_noise(particle_1, 0.1) for i in range(self.N)])
        # rospy.loginfo(self.particles)


    def odomCallback(self, msg):
        try:
            #updates particles according to motion model
            with self.lock:
                self.particles = self.motion_model.evaluate(self.particles,
                    [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
            # rospy.loginfo(self.new_particles)
        except AttributeError:
            pass

    def get_theta(self):
        '''
        input: none; directly access particle list
        output: average angle using circular averaging
        '''
        return np.arctan2(np.mean(np.sin(self.particles[:, 2])), np.mean(np.cos(self.particles[:, 2])))

    def make_pose(self, arr):
        '''
        input: [x, y, theta]
        output: creates a geometry_msgs/Point object with x, y coords as arr and fixed z = 0'''
        p = Pose()
        p.position.x = arr[0]
        p.position.y = arr[1]
        p.position.z = 0
        p.orientation.z = arr[2]
        p.orientation.w = 1
        return p

    def get_viz_msg(self):
        '''
        input: none
        output: none, directly modifies viz_msg attribute using particles list data
        '''
        self.viz_msg.header.stamp = rospy.Time.now()
        self.viz_msg.header.frame_id = "/map"
        self.viz_msg.poses = [self.make_pose(x) for x in self.particles]


    def lidarCallback(self, msg):

        try:
            #probabilities
            with self.lock:
                self.probs = self.sensor_model.evaluate(self.particles, np.array(msg.ranges))
                self.probs = self.sensor_model.normalize(self.probs)
                #weighted average for the x and y
                cumul_arr = np.cumsum(self.probs)

                self.particles = np.array([self.motion_model.add_noise(self.particles[np.min(np.argwhere(cumul_arr > np.random.random_sample())), :], 0.1) for i in range(self.N)])

                avg_dist = np.sum(self.probs * self.particles.T, axis=1)
                #separate function for theta
                avg_dist[2] = self.get_theta()

                self.get_viz_msg()
            # self.particles = np.array([self.motion_model.add_noise(avg_dist, 0.1) for i in range(self.N)])

            #populate odometry message with information
            self.odom_msg.header.stamp = rospy.get_rostime()
            self.odom_msg.header.frame_id = "map"
            self.odom_msg.child_frame_id = "base_link"
            self.odom_msg.pose.pose.position.x = avg_dist[0]
            self.odom_msg.pose.pose.position.y = avg_dist[1]
            quaternion = quaternion_from_euler(0, 0, avg_dist[2])
            self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w = quaternion
            self.odom_pub.publish(self.odom_msg)

            #publish points to topic so rviz can visualize
            self.viz_pub.publish(self.viz_msg)

            #broadcast transform from world to current location
            self.br.sendTransform((avg_dist[0], avg_dist[1], 0), quaternion,
            rospy.Time.now(), self.particle_filter_frame, "map")

        except AttributeError:
            pass


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
