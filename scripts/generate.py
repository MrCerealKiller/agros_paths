#!/usr/bin/env python

import rospy
from agros_paths import AgrosPathGenerator

if __name__ == '__main__':
	rospy.init_node('agros_path_gen')
	generator = AgrosPathGenerator()

	# Decomposer update rate
	rate = rospy.Rate(rospy.get_param('~rate', 0.2))

	# Cycle forever at fixed rate
	while not rospy.is_shutdown():
		generator.publish()
		rate.sleep()
	rospy.spin()
