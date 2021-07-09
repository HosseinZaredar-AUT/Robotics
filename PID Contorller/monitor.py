#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from gazebo_msgs.srv import GetModelState
import numpy as np
from path_generator import generate_ellipse, generate_spiral
from config import CONFIG

class Monitor():

	def __init__(self):
	
		# initiate a named node
		rospy.init_node('monitor', anonymous=True)
		rospy.loginfo('monitor initiated.')
		print('monitor initiated.')
		
		# subscribe to a service server, provided by the gazebo package to get
		# information about the state of the models present in the simulation		
		rospy.wait_for_service("gazebo/get_model_state")
		self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

		# what function to call when `ctrl + c` is issued
		rospy.on_shutdown(self.shutdown)

		# the robot's path
		self.path = []

		# pursuit points
		if CONFIG['path'] == 's':
			self.pursuit_points = generate_spiral()
		elif CONFIG['path'] == 'e':
			self.pursuit_points = generate_ellipse()

		# the robot's error in its path
		self.errors = []

		# set a read postion rate of in Hz
		self.rate = rospy.Rate(20)
				
		
	def get_robot_position(self):

		# use the service to get the robot's position
		message = self.get_ground_truth("turtlebot3_burger", "world")
		self.path.append((message.pose.position.x, message.pose.position.y))
		self.errors.append(self.distance([message.pose.position.x, message.pose.position.y]))
		print('distance =', self.errors[-1])


	def distance(self, point):

		point = np.array(point)
		distances = np.sqrt(np.sum((self.pursuit_points - point) ** 2, axis=1))
		min_dist = np.min(distances)
		return min_dist


	def shutdown(self):

		plt.plot([p[0] for p in self.pursuit_points], [p[1] for p in self.pursuit_points])

		# printing robots average error
		print(f'Average Error: {sum(self.errors) / len(self.errors)}m')

		# plotting the robot's path
		plt.plot([p[0] for p in self.path], [p[1] for p in self.path])

		plt.show()

		
if __name__ == '__main__':

	try:
		monitor = Monitor()

		# keep running until `ctrl + c` is pressed
		while not rospy.is_shutdown():
            
			# get robot's position
			monitor.get_robot_position()
			
			# wait for the specified mseconds and read position again
			monitor.rate.sleep()
	
	except:
		rospy.loginfo("monitor node terminated")
