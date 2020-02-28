#!/usr/bin/env python

import rospy
from agros_paths import AgrosPathGenerator
from agros_paths.srv import GeneratePath

if __name__ == '__main__':
	rospy.init_node('agros_path_generator')
	generator = AgrosPathGenerator()
	srv = rospy.Service('~generate_path', GeneratePath, generator.run)
	rospy.loginfo('Generate service is ready')

	# Generator update rate
	rate = rospy.Rate(rospy.get_param('~pub_rate', 0.2))

	# Cycle forever at fixed rate
	while not rospy.is_shutdown():
		generator.publish()
		rate.sleep()
	rospy.spin()
