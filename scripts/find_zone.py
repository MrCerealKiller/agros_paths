#!/usr/bin/env python

import rospy
from geodesy.utm import gridZone

def main():
	"""
	Simple utility script to find the UTM zone of WGS84 coords 
	"""

	TAG = "[find_zone.main] "

	lat = rospy.get_param('~lat', None)
	lon = rospy.get_param('~lon', None)

	# Check that at least lat and lon are provided
	missing_args = []
	if not lat:
		missing_args.append('lat (double) ')
	if not lon:
		missing_args.append('lon (double) ')

	# If missing, report and exit
	if missing_args:
		msg = ('Missing params: ')
		for arg in missing_args:
			msg = msg + arg
		rospy.logerr(TAG + msg)
		rospy.loginfo('exiting...')
		return

	try:
		lat = float(lat)
		lon = float(lon)

		rospy.loginfo(TAG + '\n' +
					  '\tLatitude:  {}\n'.format(lat) +
					  '\tLongitude: {}\n'.format(lon))

		zone, band = gridZone(lat, lon)
		rospy.loginfo(TAG + 'UTM zone of given coords:\n\n' +
					  '\t{}{}\n'.format(zone, band))

	except Error as e:
		rospy.logerr(TAG + 'Encountered error: {}'.format(e))
		rospy.loginfo('exiting...')
		return

if __name__ == '__main__':
	rospy.init_node('zone_finder')
	main()
