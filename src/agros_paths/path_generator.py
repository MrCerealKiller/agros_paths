import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
import unique_id

from geodesy.utm import UTMPoint, fromLatLong
from geographic_msgs.msg import WayPoint, GeoPoint, RouteSegment, RouteNetwork
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from shapely.geometry import LinearRing, LineString, Point, Polygon
from shapely.ops import nearest_points, split
from std_msgs.msg import Header
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
					  '\tafter setting a and b with geographic points).\n')

		self.path_pub = rospy.Publisher(
			'~path', Path, queue_size=1)
		
		self.route_pub = rospy.Publisher(
			'~route_network', RouteNetwork, queue_size=1)

		self.path = Path()
		self.route_network = RouteNetwork()
		self.route_network.id = unique_id.toMsg(unique_id.fromRandom())

		self.frame_id = rospy.get_param('~frame_id', 'odom')

		# Check UTM params -----------------------------------------------------
		self.zone = rospy.get_param('~utm_zone', None)
		if not self.zone:
			rospy.logwarn('The ROS param ~utm_zone is required by this node')

		self.band = rospy.get_param('~utm_band', None)
		if not self.zone:
			rospy.logwarn('The ROS param ~utm_band is required by this node')

		# Configure the AB line, if given --------------------------------------
		self.ab = None
		self.azimuth = None

		temp_a = rospy.get_param('~a', {}) # temp dict
		if ('latitude' in temp_a and
			'longitude' in temp_a and
			'altitude' in temp_a):
			lat = temp_a['latitude']
			lon = temp_a['longitude']
			alt = temp_a['altitude']
			self.a_geo = fromLatLong(lat, lon, alt)

		temp_b = rospy.get_param('~b', {}) # temp dict
		if ('latitude' in temp_b and
			'longitude' in temp_b and
			'altitude' in temp_b):
			lat = temp_b['latitude']
			lon = temp_b['longitude']
			alt = temp_b['altitude']
			self.b_geo = fromLatLong(lat, lon, alt)

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
					lat = point['latitude']
					lon = point['longitude']
					alt = point['altitude']
					wp = fromLatLong(lat, lon, alt)
					self.boundary_geo.append(wp)

		# Update the perimeter itself as a shapely LinearRing
		self.update_perimeter() # perimeter should be set after calling

		# Configure headlands, if given ----------------------------------------
		self.headlands = None
		self.headland_width = rospy.get_param('~headland_width', None)

		# Update the headlands as a shapely LinearRing
		self.update_headlands() # headlands should be set after calling

		# Attempt to generate boustrophedon path -------------------------------
		self.waypoints = None
		self.segments = None
		self.tool_width = rospy.get_param('~tool_width', None)
		temp_entry = rospy.get_param('~entry_position', None)
		if ('latitude' in temp_entry and
			'longitude' in temp_entry and
			'altitude' in temp_entry):
			lat = temp_entry['latitude']
			lon = temp_entry['longitude']
			alt = temp_entry['altitude']
			self.entry_geo = fromLatLong(lat, lon, alt)

		# Generate a Boustrophedon patten if all other properties are updated
		self.generate_boustrophedon()

		# Convert the Euclidean path back to geographic coordinates ------------
		self.update_path()
		self.update_geo_path()

		# Perform Visualization using matplotlib -------------------------------
		self.visualize = rospy.get_param('~visualize', False)
		if self.visualize:
			self.plot()

	# Misc. Function ===========================================================
	def set_utm_zone(self, zone):
		"""
		Sets the UTM zone for the area

		Args:
			zone: a string representing the UTM zone. Ex: "18"
		"""

		self.utm_zone = zone
		rospy.logdebug('[set_utm_zone] Successfully set UTM Zone')

	def set_utm_band(self, band):
		"""
		Sets the UTM band for the area

		Args:
			band: a string representing the UTM band. Ex: "T"
		"""

		self.utm_band = band
		rospy.logdebug('[set_utm_band] Successfully set UTM Band')

	# Functions related to the AB line =========================================
	def set_a(self, a):
		"""
		Sets the A geographic coordinate

		Args:
			a: a UTMPoint object for the A coordinate
		"""

		if isinstance(a, UTMPoint):
			self.a_geo = a
			rospy.logdebug('[set_a] Successfully set A')
		else:
			rospy.logwarn('[set_a] Given A was not configured as a ' +
						  'UTMPoint.\n\tIgnoring...')

	def set_b(self, b):
		"""
		Sets the B geographic coordinate

		Args:
			b: a UTMPoint object for the B coordinate
		"""

		if isinstance(b, UTMPoint):
			self.b_geo = b
			rospy.logdebug('[set_b] Successfully set b')
		else:
			rospy.logwarn('[set_b] Given B was not configured as a ' +
						  'UTMPoint.\n\tIgnoring...')

	def update_ab_line(self):
		"""
		Uses the members A and B (UTM coordinates) to create
		a Shapely line representing the AB line
		"""

		if (self.a_geo and self.b_geo):
			a = self.a_geo.toPoint()
			b = self.b_geo.toPoint()

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
		"""
		Sets the boundary geographic coordinates

		Args:
			boundary: a list of UTMPoint objects representing the boundary
		"""

		for point in boundary:
			if not isinstance(point, UTMPoint):
				rospy.logwarn('[set_boundary] Given boundary contained\n' +
							  '\tpoints not configured as a UTMPoint.\n' +
							  '\tIgnoring and exiting...')
				return
		# If function reaches here, boundary should be okay
		self.boundary_geo = boundary
		rospy.logdebug('[set_boundary] Successfully set boundary')

	def update_perimeter(self):
		"""
		Uses the member boundary coordinates to create a Shapely line
		representing the outer bounds of the field
		"""

		if (self.boundary_geo):
			boundary = []
			for point in self.boundary_geo:
				bound = point.toPoint()
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
		"""
		Sets the headland width in m

		Args:
			width: a number in [m] representing the headland width
		"""

		if (width > 0):
			self.headland_width = width
			rospy.logdebug('[set_headland_width] Successfully set headland ' +
						   'width')
		else:
			rospy.logwarn('[set_headland_width] width was not on (0, inf]' +
						  '\n\tIgnoring...')

	def update_headlands(self):
		"""
		Uses the member perimeter coordinates and the headland width
		to create a Shapely line representing the inner headlands bound
		"""

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
		"""
		Sets the tool width in m

		Args:
			width: a number in [m] representing the tool width
		"""

		if (width > 0):
			self.tool_width = width
			rospy.logdebug('[set_tool_width] Successfully set tool width')
		else:
			rospy.logwarn('[set_tool_width] width was not on (0, inf]' +
						  '\n\tIgnoring...')

	def set_entry(self, entry):
		"""
		Sets the entry point geographic coordinate

		Args:
			entry: a UTMPoint object for the entry point coordinate
		"""

		if isinstance(entry, UTMPoint):
			self.entry_geo = entry
			rospy.logdebug('[set_entry] Successfully set entry')
		else:
			rospy.logwarn('[set_entry] Given entry was not configured as a ' +
						  'WayPoint.\n\tIgnoring...')

	def find_equation(self, line):
		"""
		Utility class that approximates line of form y = mx + b

		Args:
			line: a Shapely line that you want the equation of
		"""

		if (isinstance(line, LineString)):
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
		"""
		Uses various members to generate a boustrophedon path. The algorithm
		is heavily commented to explain the individual steps
		"""

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
			ext_lines = []

			# The first line has a specific offset calculated above
			ext_lines.append(self.ab.parallel_offset(offset,
					direction,
	  				join_style=self.MITRE_JOIN_STYLE))

			# All others will be offset by the tool width because there is
			# one half the tool width on each side of the centreline
			while(True):
				nl = ext_lines[-1].parallel_offset(self.tool_width,
						direction,
						join_style=self.MITRE_JOIN_STYLE)

				# Check if the newline intersects the boundary
				# If it does NOT then the newline is outside the
				# working area and the loop exits without adding it
				if (nl.intersects(self.headlands)):
					ext_lines.append(nl)
				else:
					break

			# Get entry as euclidean coord and create a path to the first line
			entry = self.entry_geo.toPoint()
			entry = (entry.x, entry.y)

			self.waypoints = []
			self.waypoints.append(entry)
			last_wp = entry

			# Since these lines are still inflated split using headlands
			# and keep the inner segment (this is also repeated down below)
			self.segments = []
			allowed_area = Polygon(self.headlands)
			for line in ext_lines:
				sublns = list(split(line, self.headlands))
				
				# Items in sublns will be added to
				# self.segments and removed from sublns
				while (sublns):
					# Track the closest line segment
					# and what the distance is from that
					# segment to the last waypoint
					closest_line = None
					shortest_dist = None

					for seg in sublns:
						# The validation checks if the midpoint of the line
						# segment is contained within a polygon formed with
						# the headland line
						mid_point = seg.interpolate(0.5, normalized = True)
						if (not mid_point.within(allowed_area)):
							sublns.remove(seg)
	
						else:
							# Check distance from the last waypoint
							# to each ending coordinate of the line
							coords = list(seg.coords)
							l0 = LineString([last_wp, coords[0]]).length
							lf = LineString([last_wp, coords[-1]]).length
							
							# Set l (length) as the shortest of the two lengths
							if (l0 < lf):
								l = l0
							else:
								l = lf

							# If the shortest distance hasn't been set or
							# l is less than the shortest distance track
							# this line segment
							if ((shortest_dist is None) or (l < shortest_dist)):
								closest_line = seg
								shortest_dist = l

					# If closest_line exists, it should be added
					# to self.segments, removed from sublns, and
					# its waypoints should be added to self.waypoints
					# in the correct order (the below is slightly
					# redundant, but whatever...)
					if (closest_line):
						coords = list(closest_line.coords)
						p0 = coords[0]
						pf = coords[-1]

						l0 = LineString([last_wp, p0]).length
						lf = LineString([last_wp, pf]).length

						# Create a new line in the right direction
						if (l0 < lf):
							seg = LineString([p0, pf])
						else:
							seg = LineString([pf, p0])

						# Find the size and number of intermediary points
						# based on 1/2 of the tool width
						size_inc = seg.length / (self.tool_width / 2)
						num_inc = int(seg.length / size_inc)

						# First add the first coordinate
						self.waypoints.append(list(seg.coords)[0])
						# Then add the intermediary points
						for i in range(num_inc):
							inc = seg.interpolate(size_inc)
							self.waypoints.append(list(inc.coords)[0])
						# Finally add the last coordinate and set it as last_wp
						self.waypoints.append(list(seg.coords)[-1])
						last_wp = list(seg.coords[-1])

						self.segments.append(seg)
						sublns.remove(closest_line)

			rospy.loginfo('[generate_boustrophedon] Successfully created path')

		else:
			rospy.logwarn('[generate_boustrophedon] perimeter, headlands,\n' +
						  '\ttool width, and/or AB line has not been ' +
						  'updated.\n\tIgnoring...')

	# Functions related to updating paths ======================================
	def update_path(self):
		if (self.waypoints):
			path = Path()
			local_points = []

			# Using the entry point as the origin
			# tranlate all points to a local domain			
			for point in self.waypoints:
				x0, y0 = self.waypoints[0]
				x, y = point

				xf = x - x0
				yf = y - y0

				local_points.append((xf, yf))

			# Create a PoseStamped message for each
			# local point and add it to the path
			for point in local_points:
				x, y = point

				pose = PoseStamped()
				pose.header = Header()
				pose.header.stamp = rospy.Time.now()
				pose.header.frame_id = self.frame_id
				pose.pose.position.x = x
				pose.pose.position.y = y
				pose.pose.position.z = 0

				path.poses.append(pose)

			self.path = path

		else:
			rospy.logwarn('[update_geo_path] waypoints, zone, and/or band\n' +
						  '\thas not been set/updated.\n\tIgnoring...')

	def update_geo_path(self):
		if (self.waypoints and self.zone and self.band):
			waypoints_geo = []
			for point in self.waypoints:
				x, y = point
				geo = UTMPoint(x, y, zone=self.zone, band=self.band).toMsg()

				wp = WayPoint()
				wp.id = unique_id.toMsg(unique_id.fromRandom())
				wp.position = geo
				waypoints_geo.append(wp)

			segments_geo = []
			last_wp = waypoints_geo[0].id
			for wp in waypoints_geo:
				seg = RouteSegment()
				seg.id = unique_id.toMsg(unique_id.fromRandom())
				seg.start = last_wp
				seg.end = wp.id
				last_wp = wp.id
				segments_geo.append(seg)

			self.route_network.points = waypoints_geo
			self.route_network.segments = segments_geo

		else:
			rospy.logwarn('[update_geo_path] waypoints, zone, and/or band\n' +
						  '\thas not been set/updated.\n\tIgnoring...')

	# Functions related to point -> geographic conversion ======================
	def publish(self):
		# Publish Route
		self.route_network.header = Header()
		self.route_network.header.stamp = rospy.Time.now()
		self.route_network.header.frame_id = self.frame_id
		self.route_pub.publish(self.route_network)

		# Publish Path
		self.path.header = Header()
		self.path.header.stamp = rospy.Time.now()
		self.path.header.frame_id = self.frame_id
		self.path_pub.publish(self.path)

	# Functions related to visualization =======================================
	def plot(self):
		"""
		Uses MatPlotLib to visualize the member properties and generated path
		"""

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

		if (self.waypoints):
			waypoints_x = []
			waypoints_y = []
			for wp in self.waypoints:
				x, y = wp
				waypoints_x.append(x)
				waypoints_y.append(y)
			plt.plot(waypoints_x, waypoints_y, color="lime")

		# Finally, overlay the path segments
		if (self.segments):
			for line in self.segments:
				segs_x = []
				segs_y = []
				for x, y in list(line.coords):
					segs_x.append(x)
					segs_y.append(y)
				plt.plot(segs_x, segs_y, color='fuchsia')

		plt.axis('equal')
		plt.show()