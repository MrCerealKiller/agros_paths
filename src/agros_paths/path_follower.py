import numpy as np
import rospy
import tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from shapely.geometry import LineString
from std_msgs.msg import Header, Float64

class AgrosPathFollower(object):
	"""
	Simple path follower class to be used as a global path planner. It 
	tracks the current position and updates the next required Pose
	"""

	# Constructor ==============================================================
	def __init__(self):
		"""
		Constructor for AgrosPathFollower
		"""

		rospy.loginfo('Initializing AgrosPathFollower...\n')

		self.isSet = False # Checks if the path has been received yet
		self.path_sub = rospy.Subscriber('~path', Path, self.pathCallback)

		self.path_frame = None
		self.base_frame = rospy.get_param('~base_frame_id', 'base_link')

		self.path = None
		self.goal = None  # Index of self.path.poses that is the current goal

		self.threshold = rospy.get_param('~threshold', None)

		self._listener = tf.TransformListener()

		self.dist_pub = rospy.Publisher('~goal_dist', Float64, queue_size=1)
		self.angle_pub = rospy.Publisher('~goal_angle', Float64, queue_size=1)

	def set_threshold(self, threshold):
		"""
		Sets the acceptance threshold in m to the current goal

		Args:
			width: a number in [m] representing the acceptance threshold
		"""

		if (threshold > 0):
			self.threshold = threshold
			rospy.logdebug('[set_threshold] Successfully set threshold')
		else:
			rospy.logwarn('[set_threshold] threshold was not on (0, inf]' +
						  '\n\tIgnoring...')

	def pathCallback(self, msg):
		"""
		Called on receipt of a path message, but only used once
		to update the path and associated member variables
		"""

		if (self.isSet):
			rospy.logwarn_once('[pathCallback] received path, but path is ' + 
							   'already set. Ignoring...')
		else:
			self.path = msg
			self.path_frame = msg.header.frame_id
			
			# Set the goal to be the second index
			# because the first is acting as a
			# reference and origin
			self.goal = 1
			
			self.isSet = True

	def spin(self):
		"""
		Checks the transformation between the base link and map
		and published the distance and bearing to the next goal
		on the given path
		"""

		if (self.isSet):
			if (not self.threshold):
				rospy.logwarn_throttle(5, '[spin] Threshold not set. ' +
									   'cannot procede')
				return

			try:
				trans, rot = self._listener.lookupTransform(
					self.path_frame, self.base_frame, rospy.Time(0))

				(x, y, z) = trans
				(roll, pitch, yaw) = euler_from_quaternion(rot)

				goal_pt = self.path.poses[self.goal]
				xg = goal_pt.pose.position.x
				yg = goal_pt.pose.position.y

				direct_line = LineString([(x, y), (xg, yg)])

				# TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
				# Look ahead 1 - 3 poses to see if the robot is much
				# closer to a later point and jump to that one... This should
				# recover from a skipped point
				# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

				# If the goal point is close increment to
				# a new goal point and recalculate the line
				if (direct_line.length < self.threshold):
					self.goal = self.goal + 1

					goal_pt = self.path.poses[self.goal]
					xg = goal_pt.pose.position.x
					yg = goal_pt.pose.position.y

					direct_line = LineString([(x, y), (xg, yg)])

				dx = (xg - x)
				dy = (yg - y)

				goal_dist = direct_line.length
				goal_bearing = np.arctan2(dx, dy)

				self.dist_pub.publish(goal_dist)
				self.angle_pub.publish(goal_bearing)

				return

			except (tf.LookupException,
					tf.ConnectivityException,
					tf.ExtrapolationException) as e:
				rospy.logwarn_throttle(2, '[spin]' + e.message)
		else:
			rospy.logwarn_throttle(5, '[spin] Waiting for initial path...')