#!/usr/bin/env python2

import numpy as np
import rospy
from nav_msgs.msg import Odometry

class TestParticleFilter:

    def __init__(self):
        #rospy.spin()
        #subscribe to odometry, particle filter result
        #rospy.loginfo("init 1")
        rospy.init_node("test_particle_filter")
        self.data_file = open('test.txt', 'w')
        self.pf_data = [0, 0, 0]
        self.odom_data = [0, 0, 0]
        self.odom_msg = Odometry()
        self.odom_pub = rospy.Publisher("/odom/noisy", Odometry, queue_size = 1)
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odomCallback, queue_size=1)
        self.pf_odom_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.pfOdomCallback, queue_size=1)
        #self.compare_truth_to_pf()
        print("init")
        #get x, y, theta from odometry sub
        #get output from add_noise
        
        #self.odom_pub = rospy.Publisher("/odom/noisy", Odometry, queue_size = 1)
        #self.odom_msg = Odometry()
        #self.add_noise(odom_msg
        #noisy_data = self.add_noise(self.od_data, eta)
       # self.compare_truth_to_pf()
        #rospy.loginfo("init 2")
        #publish noisy odometry data

    def add_noise(self, particle, eta):
        """
        Add some random noise with noise-factor eta to the particles
        """
        x = particle[0] + np.random.normal(0,abs(eta*particle[0]))
        y = particle[1] + np.random.normal(0,abs(eta*particle[1]))
        th = particle[2] + np.random.normal(0,abs(eta*particle[2]))
        #rospy.loginfo("add_noise")
        return [x,y,th]

    def odomCallback(self, msg):
       # print("odomc")
            #updates particles according to motion model
        self.od_data = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        eta = .1;
        print("Og odom data: " + str(self.od_data[0]) + ", " + str(self.od_data[1]) + ", " + str(self.od_data[2]) )
        noisy_data = self.add_noise(self.od_data, eta)
        self.odom_msg.twist.twist.linear.x = noisy_data[0]
        self.odom_msg.twist.twist.linear.y = noisy_data[1]
        self.odom_msg.twist.twist.angular.z = noisy_data[2]
        self.odom_pub.publish(self.odom_msg)
        #rospy.loginfo(0)

    def pfOdomCallback(self, msg):
        #rospy.loginfo("pfodom"
        #print(msg.twist.twist.linear.x)
        self.pf_data =[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z] 
        self.compare_truth_to_pf()

    def compare_truth_to_pf(self):
        #find the error between the ground truth (published to base_link) and particle filter (pb base_link_pf)
        print("compare")
        pf_data = self.pf_data
        true_data = self.odom_data
        x_diff = abs(pf_data[0] - true_data[0])
        y_diff = abs(pf_data[1] - true_data[1])
        t_diff = abs(pf_data[2] - true_data[2])
        print(pf_data[1])
        #data_file = open('test.txt', 'w')
        self.data_file.write(str(x_diff) + "\n")

if __name__ == "__main__":
    rospy.loginfo("main")
 #   rospy.init_node("test_particle_filter")
    print("node created")
    tpf = TestParticleFilter()
   # odom_topic = rospy.get_param("~odom_topic", "/odom")
  #  rospy.Subscriber(odom_topic, Odometry,
 #                                     tpf.odomCallback, queue_size=1)
#    rospy.Subscriber("/pf/pose/odom", Odometry, tpf.pfOdomCallback, queue_size=1)
    #tpf = TestParticleFilter()
    rospy.spin()


