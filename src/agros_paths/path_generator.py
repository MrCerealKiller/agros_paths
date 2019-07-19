import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
import unique_id

from geodesy import wu_point as WP
from geographic_msgs.msg import WayPoint, GeoPoint
from shapely.geometry import LinearRing, LineString, Point
from shapely.ops import nearest_points, split
from uuid_msgs.msg import UniqueID

class AgrosPathGenerator(object):
	"""
	Path generation class that implements the ROS geodesy package and shapely's
	geometric analysis to create a boustrophedon path
	"""

	OFFSET_DIRECTION = 'left' # With CCW, + is on LHS of closed line
	MITRE_JOIN_STYLE = 2 	  # Shapely constant --> "MITRED"

	# Constructor ==============================================================
	def __init__(self):
		"""
		Constructor for AgrosPathGenerator
		"""
		rospy.loginfo('Initializing AgrosPathGenerator...\n\n'
					  '\t(Note: this class polls rosparams on\n' +
					  '\tconstruction. If you are setting any parameters\n'+
					  '\tmanually, please review the API and call the\n' +
					  '\tright functions (e.x. call update_ab_line()\n' +
					  '\tafter setting a and b with geographic points).')

		# Configure the AB line, if given --------------------------------------
		self.ab = None
		self.azimuth = None

		temp_a = rospy.get_param('~a', {}) # temp dict
		if ('latitude' in temp_a and
			'longitude' in temp_a and
			'altitude' in temp_a):
			wp = WayPoint()
			wp.id = unique_id.toMsg(unique_id.fromRandom())
			wp.position.latitude = temp_a['latitude']
			wp.position.longitude = temp_a['longitude']
			wp.position.altitude = temp_a['altitude']
			self.a_geo = wp

		temp_b = rospy.get_param('~b', {}) # temp dict
		if ('latitude' in temp_b and
			'longitude' in temp_b and
			'altitude' in temp_b):
			wp = WayPoint()
			wp.id = unique_id.toMsg(unique_id.fromRandom())
			wp.position.latitude = temp_b['latitude']
			wp.position.longitude = temp_b['longitude']
			wp.position.altitude = temp_b['altitude']
			self.b_geo = wp

		# Update the AB line itself as a shapely LineString
		self.update_ab_line() # ab and ab_angle should be set after calling

		# Configure the boundaries, if given -----------------------------------
		self.perimeter = None

		temp_boundary = rospy.get_param('~boundary', None) # temp dict
		if 'points' in temp_boundary:
			self.boundary_geo = []
			for point in temp_boundary['points']:
				if ('latitude' in point and
					'longitude' in point and
					'altitude' in point):
					wp = WayPoint()
					wp.id = unique_id.toMsg(unique_id.fromRandom())
					wp.position.latitude = point['latitude']
					wp.position.longitude = point['longitude']
					wp.position.altitude = point['altitude']
					self.boundary_geo.append(wp)

		# Update the perimeter itself as a shapely LinearRing
		self.update_perimeter() # perimeter should be set after calling

		# Configure headlands, if given ----------------------------------------
		self.headlands = None
		self.headland_width = rospy.get_param('~headland_width', None)

		# Update the headlands as a shapely LinearRing
		self.update_headlands() # headlands should be set after calling

		# Attempt to generate boustrophedon path -------------------------------
		self.path = None
		self.tool_width = rospy.get_param('~tool_width', None)
		temp_entry = rospy.get_param('~entry_position', None)
		if ('latitude' in temp_entry and
			'longitude' in temp_entry and
			'altitude' in temp_entry):
			wp = WayPoint()
			wp.id = unique_id.toMsg(unique_id.fromRandom())
			wp.position.latitude = temp_entry['latitude']
			wp.position.longitude = temp_entry['longitude']
			wp.position.altitude = temp_entry['altitude']
			self.entry_geo = wp

		# Generate a Boustrophedon patten if all other properties are updated
		self.generate_boustrophedon() # 

		# Convert the Euclidean path back to geographic coordinates ------------
		self.waypoints = None
		self.segments = None

		# Perform Visualization using matplotlib -------------------------------
		self.visualize = True # TODO Replace with a rosparam
		self.plot()

	# Functions related to the AB line =========================================
	def set_a(self, a):
		if isinstance(a, WayPoint):
			self.a_geo = a
			rospy.logdebug('[set_a] Successfully set A')
		else:
			rospy.logwarn('[set_a] Given A was not configured as a ' +
						  'WayPoint.\n\tIgnoring...')

	def set_b(self, b):
		if isinstance(b, WayPoint):
			self.b_geo = b
			rospy.logdebug('[set_b] Successfully set b')
		else:
			rospy.logwarn('[set_b] Given B was not configured as a ' +
						  'WayPoint.\n\tIgnoring...')

	def update_ab_line(self):
		if (self.a_geo and self.b_geo):
			a = WP.WuPoint(self.a_geo).toPoint()
			b = WP.WuPoint(self.b_geo).toPoint()

			self.ab = LineString([(a.x, a.y, a.z), (b.x, b.y, b.z)])

			dx = (b.x - a.x)
			dy = (b.y - a.y)

			dist = self.ab.length
			angle = np.arctan2(dx, dy)
			if (angle >= 0):
				self.azimuth = angle
			else:
				self.azimuth = (angle + (2 * math.pi))

			rospy.loginfo('[update_ab_line] Updated AB Line:' +
						  '\n\tDistance: {}'.format(dist) +
						  '\n\tAngle: {}'.format(math.degrees(self.azimuth)))
		else:
			rospy.logwarn('[update_ab_line] a_geo and/or b_geo were' +
						  'not set.\n\tIgnoring...')

	# Functions related to the boundary ========================================
	def set_boundary(self, boundary):
		for point in boundary:
			if not isinstance(point, WayPoint):
				rospy.logwarn('[set_boundary] Given boundary contained\n' +
							  '\tpoints not configured as a WayPoint.\n' +
							  '\tIgnoring and exiting...')
				return
		# If function reaches here, boundary should be okay
		self.boundary_geo = boundary
		rospy.logdebug('[set_boundary] Successfully set boundary')

	def update_perimeter(self):
		if (self.boundary_geo):
			boundary = []
			for point in self.boundary_geo:
				bound = WP.WuPoint(point).toPoint()
				boundary.append((bound.x, bound.y, bound.z))
			self.perimeter = LinearRing(boundary)

			if (not self.perimeter.is_valid):
				rospy.logwarn('[update_perimeter] The given boundary\n' +
							  '\tcoordinates cause an intersection.\n' +
							  '\tPlease check the order of your\n' +
							  '\tcoordinates and try again. Resetting...')
				self.perimeter = None
				return

			if (not self.perimeter.is_ccw):
				rospy.logwarn('[update_perimeter] The given boundary\n' +
							  '\tcoordinates are not given in\n' +
							  '\tcounter-clockwise order. Please check your\n' +
							  '\tcoordinates and try again. Resetting...')
				self.perimeter = None
				return

			rospy.loginfo('[update_perimeter] Updated the bounds with ' +
						  '{} coords'.format(len(self.perimeter.coords) - 1))
		else:
			rospy.logwarn('[update_perimeter] boundary_geo was not set with ' +
						  'geographic coords.\n\tIgnoring...')

	# Functions related to the boundary ========================================
	def set_headland_width(self, width):
		if (width > 0):
			self.headland_width = width
			rospy.logdebug('[set_headland_width] Successfully set headland ' +
						   'width')
		else:
			rospy.logwarn('[set_headland_width] width was not on (0, inf]' +
						  '\n\tIgnoring...')

	def update_headlands(self):
		if (self.headland_width and self.perimeter):
			self.headlands = self.perimeter.parallel_offset(
				self.headland_width,
				self.OFFSET_DIRECTION,
				join_style=self.MITRE_JOIN_STYLE)

			rospy.loginfo('[update_headlands] Updated headlands at a width ' +
						  'of {}'.format(self.headland_width))
		else:
			rospy.logwarn('[update_headlands] headland_width and/or\n' +
						  '\tboundary has not been updated.\n\tIgnoring...')

	# Functions related to the boundary ========================================
	def set_tool_width(self, width):
		if (width > 0):
			self.tool_width = width
			rospy.logdebug('[set_tool_width] Successfully set tool width')
		else:
			rospy.logwarn('[set_tool_width] width was not on (0, inf]' +
						  '\n\tIgnoring...')

	def set_entry(self, entry):
		if isinstance(entry, WayPoint):
			self.entry_geo = entry
			rospy.logdebug('[set_entry] Successfully set entry')
		else:
			rospy.logwarn('[set_entry] Given entry was not configured as a ' +
						  'WayPoint.\n\tIgnoring...')

	def find_equation(self, line):
		if (isinstance(line, LineString)):
			"""
			Approximates line of form y = mx + b
			"""

			# Get the given euclidean coordinates
			x0, y0, z0 = line.coords[0]
			x1, y1, z0 = line.coords[-1]
			
			# Determine the differential components
			dx = (x1 - x0)
			dy = (y1 - y0)

			# Solve for m and b (up to computer's resolution)
			m = (dy / dx)
			b = y0 - (m * x0)

			rospy.loginfo('[find_equation] Equation found is:\n' +
						  '\ty = ({})x + ({})'.format(m, b))

			return m, b
		else:
			rospy.logwarn('[find_equation] The given line was not a\n' +
						  '\tLineString object.\n' +
						  '\tIgnoring...')
			return None, None

	def generate_boustrophedon(self):
		if (self.perimeter and self.headlands and self.ab and self.tool_width):
			# Find m and b and exit if there is an error
			m, b = self.find_equation(self.ab)
			if m is None:
				rospy.logwarn('[generate_boustrophedon] Could not retrieve\n' +
							  '\tlinear equation. Aborting path generation...')
				return

			# Bounds of perimeter
			(p_minx, p_miny, p_maxx, p_maxy) = self.perimeter.bounds
			# Bounds of initial AB line
			(ab_minx, ab_miny, ab_maxx, ab_maxy) = self.ab.bounds

			# Copy the original set of coordinates
			new_coords = list(self.ab.coords)
			if (p_minx < ab_minx):
				# Add a coordinate at p_minx
				y = (m * p_minx) + b
				new_coords.append((p_minx, y, 0))
			if (p_miny < ab_miny):
				# Add a coordinate at p_miny
				x = (p_miny - b) / m
				new_coords.append((x, p_miny, 0))
			if (p_maxx > ab_maxx):
				# Add a coordinate at p_maxx
				y = (m * p_maxx) + b
				new_coords.append((p_maxx, y, 0))
			if (p_maxy > ab_maxy):
				# Add a coordinate at p_maxy
				x = (p_maxy - b) / m
				new_coords.append((x, p_maxy, 0))

			# Sort the coordinates by their x value
			# Then take the first and last one (most extreme) as new ab line
			new_coords.sort()
			self.ab = LineString([new_coords[0], new_coords[-1]])

			# Find the initial offset between the AB line and the headlands
			# pab --> PointAB
			# ph --> PointHeadlands
			pab, ph = nearest_points(self.ab, self.headlands)
			offset = LineString([pab, ph]).length + (self.tool_width / 2.0)

			"""
			So we need to choose the right direction for the offset here...

			We have two possibilities:
			
			Negative Slope:		Positive Slope:
			|  *				|	   /
			|   \				|     /
			|    \				|    /
			|     \				|   / 
			|      \			|  *
			---------------		---------------

			Since we odered line coordinates by x-value, we know the line
			always goes from right to left. By observation, you can notice
			that if (y_h - y_pab) is positive (i.e. the headlands are above
			the line), then the offset will always be to the left and
			vice-versa.
			"""
			if ((ph.y - pab.y) > 0):
				direction = 'left'
			else:
				direction = 'right'

			# Create an empty list to add the lines to
			path_lines = []

			# The first line has a specific offset calculated above
			fl = self.ab.parallel_offset(offset,
					direction,
	  				join_style=self.MITRE_JOIN_STYLE)

			# Since this is still inflated split using headlands and only
			# keep the inner segment (this is also repeated down below)
			subnls = split(fl, self.headlands)
			for seg in subnls:
				if (not seg.intersects(self.perimeter)):
					fl = seg # Replace fl with inner segment

			# Get entry as euclidean coord and create a path from entry to fl
			entry = WP.WuPoint(self.entry_geo).toPoint()
			entry = Point(entry.x, entry.y, entry.z)
			embark_path = LineString(nearest_points(entry, fl))

			# Add both the initial path and the first line to the paths
			path_lines.append(embark_path)
			path_lines.append(fl)

			# All others will be offset by the tool width because there is
			# one half the tool width on each side of the centreline
			while(True):
				nl = path_lines[-1].parallel_offset(self.tool_width,
						direction,
						join_style=self.MITRE_JOIN_STYLE)

				# Check if the newline intersects the boundary
				# If it does NOT then the newline is outside the
				# working area and the loop exits without adding it
				if (nl.intersects(self.headlands)):
					subnls = split(nl, self.headlands)
					for seg in subnls:
						if (not seg.intersects(self.perimeter)):
							path_lines.append(seg)
				else:
					break

			# TODO Temporary!!!! This will be replaced with an algorithm for connecting the individual line segments into one path
			self.path = path_lines

			rospy.loginfo('[inflate_ab_line] inflated ab line to reach\n' +
						  '\tperimeter bounds')
		else:
			rospy.logwarn('[generate_boustrophedon] perimeter, headlands,\n' +
						  '\ttool width, and/or AB line has not been ' +
						  'updated.\n\tIgnoring...')

	# Functions related to visualization =======================================
	def plot(self):
		# Draw the polygon representing the perimeter first
		if (self.perimeter):
			perimeter_x = []
			perimeter_y = []
			for x, y, z in list(self.perimeter.coords):
				perimeter_x.append(x)
				perimeter_y.append(y)
			plt.plot(perimeter_x, perimeter_y)

		# Draw the polygon representing the headlands next
		if (self.headlands):
			headlands_x = []
			headlands_y = []
			for x, y in list(self.headlands.coords):
				headlands_x.append(x)
				headlands_y.append(y)
			plt.plot(headlands_x, headlands_y)

		# Draw a line segment representing the AB line next
		if (self.ab):
			ab_x = []
			ab_y = []
			for x, y, z in list(self.ab.coords):
				ab_x.append(x)
				ab_y.append(y)
			plt.plot(ab_x, ab_y)

		# Finally, overlay the path
		for line in self.path:
			if (self.path):
				path_x = []
				path_y = []
				for x, y in list(line.coords):
					path_x.append(x)
					path_y.append(y)
				plt.plot(path_x, path_y)

		plt.axis('equal')
		plt.show()