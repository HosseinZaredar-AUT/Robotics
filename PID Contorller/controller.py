#! /usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_generator import generate_ellipse, generate_spiral
from config import CONFIG
import time

class Controller():

    def __init__(self):

        # initiate the node
        rospy.init_node('controller', anonymous=False)
        rospy.loginfo('controller initiated.')
        print('controller initiated.')

        time.sleep(5)

        # what function to call when `ctrl + c` is issued
        rospy.on_shutdown(self.shutdown)

        # create a Publisher object to control the robot
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # creating the square that the robot needs to follow
        self.pursuit_points = self.create_pursuit_points()
        self.current_pursuit_point = 0

        # subscribe to topic /odom published by the robot base to read odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)

        # varibles to store the current position of the robot
        # which is calculated from odometry data
        self.current_position_x = 0
        self.current_position_y = 0
        self.theta = 0

        # integral term
        self.int_e_p = 0

        # create a variable of type Twist for velocity of the robot
        self.vel = Twist()

        # set a publish velocity rate of in Hz
        self.rate = rospy.Rate(20)


    def send_velocity_cmd(self):

        # if we reach the current pursuit point, we set it to the next one
        distance = math.sqrt((self.pursuit_points[self.current_pursuit_point][0] - self.current_position_x) ** 2 + 
            (self.pursuit_points[self.current_pursuit_point][1] - self.current_position_y) ** 2)

        k_p = CONFIG['k_p']          # proportional coefficient for velocity
        k_i = CONFIG['k_i']          # integral coefficient for velocity
        ds = CONFIG['ds']            # d_star used in velocity controller
        max_vel = CONFIG['max_vel']  # maximum linear velocity of the robot

        if distance < 2 * ds:
            self.current_pursuit_point = (self.current_pursuit_point + 1) % len(self.pursuit_points)
            self.int_e_p = 0

        # distance error
        e_p = distance - ds

        # calculating the integral of the error
        self.int_e_p += e_p * 0.05

        # linear velocity of the robot
        self.vel.linear.x = min(max_vel, k_p * e_p + k_i * self.int_e_p)

        k_theta = CONFIG['k_theta']  # proportional coefficient for theta

        # the target angle
        theta_star = math.atan2(self.pursuit_points[self.current_pursuit_point][1] - self.current_position_y,
            self.pursuit_points[self.current_pursuit_point][0] - self.current_position_x)
        
        # the change the robot needs to make in its angle
        delta_theta = theta_star - self.theta
        if delta_theta > math.pi:
            delta_theta = delta_theta - 2 * math.pi
        elif delta_theta < -math.pi:
            delta_theta = delta_theta + 2 * math.pi

        # publishing linear and angular velocity to the robot
        self.vel.angular.z = k_theta * delta_theta
        self.vel_pub.publish(self.vel)


    def create_pursuit_points(self):
        if CONFIG['path'] == 's':
            points = generate_spiral()
        elif CONFIG['path'] == 'e':
            points = generate_ellipse()
        return points


    def callback_odometry(self, msg):

        # reading the robot's position from odometry
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y

        # reading the robots heading form odometry
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = yaw


    def shutdown(self):

        rospy.loginfo('Stopping the robot.')

        # stop the robot
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.vel_pub.publish(self.vel)

        # making sure the robot receives the command before exitting
        rospy.sleep(1)


if __name__ == '__main__':

	try:
		controller = Controller()
		
		# keep running until `ctrl + c` is pressed
		while not rospy.is_shutdown():
            
			# send velocity commands to the robots
			controller.send_velocity_cmd()
			
			# wait for the specified mseconds and publish velocity again
			controller.rate.sleep()
	
	except:
		rospy.loginfo("controller node terminated")