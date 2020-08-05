#!/usr/bin/env python

import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct


class MissionPlanner(object):
	def __init__(self):
		rospy.init_node('mission_planner')
		# Subscriber
		rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, self.process_pcl) # Point Cloud published by RTAB-Map
		print('Mission Planner listening for point cloud..')
	
		self.goal_pub = rospy.Publisher('/rtabmap/goal', PoseStamped, queue_size=1)
		rospy.spin()

	def process_pcl(self, msg):
		print('Received Point Cloud.')
		goal_pose = PoseStamped()
		goal_pose.header.stamp = rospy.Time.now()
		goal_pose.header.frame_id = "map"

		white_points = [] # proof of concept to get all points with the same color
		for p in pc2.read_points(msg, skip_nans=True, field_names=("x","y","z","rgb")):
			color_detected = struct.unpack('I', struct.pack('f', p[3]))[0]
			rgb_values = [int((color_detected & 0x00FF0000) >> 16), int((color_detected & 0x0000FF00) >> 8), int((color_detected & 0x000000FF))]
			if rgb_values[0] == rgb_values[1] == rgb_values[2]:
				white_points.append(p)
		# Should process white points here but for now choose first one
		goal_pose.pose.position.x = white_points[0][0]
		goal_pose.pose.position.y = white_points[0][1]
		goal_pose.pose.position.z = white_points[0][2]
		# Dummy orientation for now
		goal_pose.pose.orientation.x = 0
		goal_pose.pose.orientation.y = 0
		goal_pose.pose.orientation.z = 0
		goal_pose.pose.orientation.w = 1

		self.goal_pub.publish(goal_pose)


if __name__ == '__main__':
	try:
		MissionPlanner()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start mission_planner node.')
