#!/usr/bin/env python

import yaml
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory

class PathPlanner(object):
	def __init__(self):

		# Subscribers
		sub1 = rospy.Subscriber('/labelled_global_pcl', PointCloud2, self.process_pcl) # Global Point Cloud published by SLAM
		sub2 = rospy.Subscriber('/depth_set_point', Float32, self.go_to_depth) # Depth set point (usually published by mission planner tasks)
		sub3 = rospy.Subscriber('/2D_set_point', Point, self.go_to_2D_point) # 2D set point (usually published by object detection module)
		sub4 = rospy.Subscriber('/go_to_landmark', Point, self.go_to_landmark) # RGB unique landmark identifier (x, y, z represent r, g, b respectively) (usually published by mission planner tasks)

		
		print('Path Planner is listening for data..')

		self.traj_pub = rospy.Publisher('/trajectory', JointTrajectory, queue_size=1) # Trajectory publisher

		# Load configuration file here
		# config_string = rospy.get_param("/path_planner_config")
		# self.config = yaml.load(config_string)

		rospy.init_node('path_planner')
		rospy.spin()

	def process_pcl(self, msg):
		print('Received Point Cloud.')
	
	def go_to_depth(self, msg):
		print('Received depth set point.')

	def go_to_2D_point(self, msg):
		print('Received 2D set point.')

	def go_to_landmark(self, msg):
		print('Received landmark set point.')

if __name__ == '__main__':
	try:
		PathPlanner()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start path_planner node.')
