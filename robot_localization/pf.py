#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.norm_w: float = 1.0
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # TODO: define additional helper functions if needed

    def turn(self, delta_theta):
        self.theta += delta_theta

    def drive(self, delta_dist):
        self.x += delta_dist*np.cos(self.theta)
        self.y += delta_dist*np.sin(self.theta)

class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf') # type: ignore
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        # TODO: define additional constants if needed

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud: list[Particle] = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1) # type: ignore
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):  # type: ignore
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        # print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # TODO: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin
        x_avg = 0.0
        y_avg = 0.0
        cos_avg = 0.0
        sin_avg = 0.0
        for p in self.particle_cloud:
            x_avg += (p.x)*(p.w)
            y_avg += (p.y)*(p.w)
            cos_avg += np.cos(p.theta)*(p.w)
            sin_avg += np.sin(p.theta)*(p.w)
        theta_avg = np.arctan2(sin_avg, cos_avg)
        print(x_avg, y_avg, theta_avg)
        self.robot_pose = Particle(x_avg, y_avg, theta_avg, 1).as_pose()

        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return
        theta1 = np.arctan2(delta[1], delta[0]) - old_odom_xy_theta[2]
        dist = np.sqrt(delta[0]**2 + delta[1]**2)
        theta2 = new_odom_xy_theta[2] - np.arctan2(delta[1], delta[0])
        for p in self.particle_cloud:
            p.turn(theta1+np.random.randn()*0.01)
            p.drive(dist+np.random.randn()*0.1)
            p.turn(theta2+np.random.randn()*0.01)
        

        # TODO: modify particles using delta

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """

        # the number of particles to keep from the original particles, a proportion from 0-1
        amount_to_keep = 0.25

        # get a list of the weights of the particles and sort them, keeping track of the indicies
        weights = []
        for particle in self.particle_cloud:
            weights.append(particle.w)
        sorted_indexes = np.argsort(weights)

        # get the n particles with the highest weights, n being determined by the amount_to_keep
        worst_keeper_index = round(self.n_particles * (1-amount_to_keep))
        best_particle_indexes  = sorted_indexes[worst_keeper_index:self.n_particles-1]
        best_particles = [self.particle_cloud[i] for i in best_particle_indexes]
        self.particle_cloud = best_particles

        # normalize the weights of the remaining particles
        self.normalize_particles()

        # find the cumulative probability for that particles can be selected based on their probability with a uniformly distributed random number
        probability = np.array([particle.norm_w for particle in self.particle_cloud])
        cumulative_probability = probability.cumsum()

        # create a new set of particles based off the old particles, prefering more likely particles
        new_particles: list[Particle] = []
        for i in range(self.n_particles):
            selection_array = np.array([cumulative_probability > np.random.rand()])
            selection_index = np.searchsorted(selection_array.flatten(), 1)
            selected_matching_particle: Particle = self.particle_cloud[selection_index]
            
            position_noise = 1/12   # The spread of the x and y positions, should keep most points within 0.5 meter circle centered on mean x and y
            angle_noise = np.pi/24  # The spread of the angles, should keep most points within 45 degrees centered on mean (22.5 degrees to either side)

            # select parameters for the new partcile centered at the chosen partcile but with some noise
            x = np.random.normal(selected_matching_particle.x, position_noise)
            y = np.random.normal(selected_matching_particle.y, position_noise)
            theta = np.random.normal(selected_matching_particle.theta, angle_noise)

            new_particles.append(Particle(x, y, theta))
        
        # replace the old particles with the new particles
        self.particle_cloud = new_particles

        #
        # Thoughts: Should it really prefer more likely particles? It might be interesting to try
        # the opposite: finding the inverse weights of each and then selecting based on that. This
        # might make it so that there would be a stable equalibrium for a multimodal distribution.
        # This also only works when only the most likely particles are selected, otherwise it
        # would be a pretty terrible selection system.
        #

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """
        # TODO: implement this
        for p in self.particle_cloud:
            for r_i, theta_i in zip(r, theta):
                if np.isinf(r_i):
                    continue
                x = r_i*np.cos(p.theta + theta_i)
                y = r_i*np.sin(p.theta + theta_i)
                obstacle_dist = self.occupancy_field.get_closest_obstacle_distance(x,y)
                if not np.isnan(obstacle_dist):
                    if obstacle_dist < 0.15:
                        p.w += 1


        self.normalize_particles()
        pass

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        self.particle_cloud = []

        position_sigma = 1/6    # The spread of the x and y positions, should keep most points within 1 meter circle centered on mean x and y
        angle_sigma = np.pi/12  # The spread of the angles, should keep most points within 45 degrees left or right of mean

        # create n_particles particles
        for i in range(self.n_particles):
            # sample starting values with a normal distribution
            x = np.random.normal(xy_theta[0], position_sigma)
            y = np.random.normal(xy_theta[1], position_sigma)
            theta = np.random.normal(xy_theta[2], angle_sigma)

            # add a particle to the particle cloud with the random parameters
            self.particle_cloud.append(Particle(x, y, theta))

        self.normalize_particles()
        self.update_robot_pose()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # Find the sum of the weights of all of the particles
        total_weight = 0
        for p in self.particle_cloud:
            total_weight += p.w
        
        # Divide the weight of each particle by the sum of all particle weights to normalize
        for p in self.particle_cloud:
            p.norm_w = p.w / total_weight

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))  # type: ignore
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
