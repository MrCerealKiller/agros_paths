#!/usr/bin/env python

import rospy
from agros_paths import AgrosPathFollower

if __name__ == '__main__':
	rospy.init_node('agros_path_follower')
	follower = AgrosPathFollower()

	# Follower update rate
	rate = rospy.Rate(rospy.get_param('~rate', 20))

	# Cycle forever at fixed rate
	while not rospy.is_shutdown():
		follower.spin()
		rate.sleep()
	rospy.spin()
