#!/usr/bin/env python

import rospy
from agros_paths import AgrosPathGenerator

if __name__ == '__main__':
	rospy.init_node('path_planner')
	generator = AgrosPathGenerator()
