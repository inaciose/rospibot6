#!/usr/bin/env python

# OccupancyGrid publisher
# author: Riccardo Giubilato
# mail: riccardo.giubilato@gmail.com
# year: 2017
# ============================================
# subscribed topics:
#	/car/scan - contains LaserScan messages
# published topics:
#   /car/map - contains OccupancyGrid messages
# listened tf:
#   /car_base_link to /world
#	/sonar_link to /car_base_link
#   /map to /world
# ============================================

import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

import time

# initialize node
rospy.init_node('later_to_occupancy_grid_node')

# listener of transforms between the car_base_link and the world frame
car_pose = tf.TransformListener()

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'odom'
resolution = 0.1
width = 1000
height = 1000

# Initialize car pose relative to world
x_car = 0.0
y_car = 0.0

# square size of the car footprint [m]
footprint = 0.3

# Map update rate (defaulted to 5 Hz)
rate = 5.0

# Range data
#car_range = 0.0
car_range = []

def set_free_cells(grid, position, size):
	# set free the cells occupied by the car
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# size: 			r     radius of the footprint
	global resolution

	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2

	# set the roi to 1: known free positions
	for i in range(-size//2, size//2):
		for j in range(-size//2, size//2):
			grid[int(i + off_x), int(j + off_y)] = 1

def set_obstacle(grid, position, orientation, position_sonar, quaternion_sonar, car_range):
	# set the occupied cells when detecting an obstacle
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# orientation:      quaternion, orientation of the car
	global resolution

	scanItens = len(car_range)

	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2

	euler = tf.transformations.euler_from_quaternion(orientation)

	rospy.loginfo("scanItens: %d",scanItens)

	if(scanItens == 0):
		return

        #for i in range(0,scanItens):
	ix = -1;
	while ix < scanItens - 1:

		ix = ix + 1
		rospy.loginfo("scan i: %d a: %d d: %f", scanItens, ix, car_range[ix])

		if not car_range[ix] == 0.0:

			rotMatrix = numpy.array([[numpy.cos(euler[2]),   numpy.sin(euler[2])],
			                     [-numpy.sin(euler[2]),  numpy.cos(euler[2])]])
			obstacle = numpy.dot(rotMatrix,numpy.array([ix, (car_range[ix] + position_sonar[0]) // resolution])) + numpy.array([off_x,off_y])

			#rospy.loginfo("FOUND OBSTACLE AT: x:%f y:%f", obstacle[0], obstacle[1])
        	        rospy.loginfo("hit %f %f", obstacle[0], obstacle[1])

			# set probability of occupancy to 100 and neighbour cells to 50
			grid[int(obstacle[0]), int(obstacle[1])] = int(100)
			if  grid[int(obstacle[0]+1), int(obstacle[1])]   < int(1):
				grid[int(obstacle[0]+1), int(obstacle[1])]   = int(50)
			if  grid[int(obstacle[0]), 	 int(obstacle[1]+1)] < int(1):
				grid[int(obstacle[0]),   int(obstacle[1]+1)] = int(50)
			if  grid[int(obstacle[0]-1), int(obstacle[1])]   < int(1):
				grid[int(obstacle[0]-1), int(obstacle[1])]   = int(50)
			if  grid[int(obstacle[0]),   int(obstacle[1]-1)] < int(1):
				grid[int(obstacle[0]),   int(obstacle[1]-1)] = int(50)

			t = 0.5
			i = 1
			free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
			while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
				grid[int(free_cell[0]), int(free_cell[1])] = int(0)
				free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
				i = i+1;

	rospy.loginfo("Scan OUT")
	#i = i + 10

def callback_range(msg):
	# callback range
	global car_range
	#car_range = msg.ranges[0]
        car_range = msg.ranges


# Subscribers
range_sub = rospy.Subscriber("/scan", LaserScan, callback_range)

# Publishers
occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)


# main function
if __name__ == '__main__':

	# set grid parameters
	if rospy.has_param("occupancy_rate"):
		rate = rospy.get_param("occupancy_rate")

	if rospy.has_param("grid_resolution"):
		resolution = rospy.get_param("grid_resolution")

	if rospy.has_param("grid_width"):
		width = rospy.get_param("grid_width")

	if rospy.has_param("grid_height"):
		height = rospy.get_param("grid_height")

	# fill map_msg with the parameters from launchfile
	map_msg.info.resolution = resolution
	map_msg.info.width = width
	map_msg.info.height = height
	map_msg.data = range(width*height)

	# initialize grid with -1 (unknown)
	grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int),
	         dtype=numpy.int)
	grid.fill(int(-1))

	# set map origin [meters]
	map_msg.info.origin.position.x = - width // 2 * resolution
	map_msg.info.origin.position.y = - height // 2 * resolution

        rate = 1000
	loop_rate = rospy.Rate(rate)

	while not rospy.is_shutdown():

		scanItens = len(car_range)
		rospy.loginfo("L")

                if (scanItens == 0):
			time.sleep(0.5)
			continue

		rospy.loginfo("Ls")

		try:
			t = car_pose.getLatestCommonTime("/base_link", "/odom")
			position, quaternion = car_pose.lookupTransform("/odom", "/base_link", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		try:
			t = car_pose.getLatestCommonTime("/base_link", "/neato_laser")
			position_sonar, quaternion_sonar = car_pose.lookupTransform("/base_link", "/neato_laser", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

                rospy.loginfo("pos: %f %f", position[0], position[1])

		# write 0 (null obstacle probability) to the free areas in grid
		set_free_cells(grid, position, int(footprint//resolution))

		# write p>0 (non-null obstacle probability) to the occupied areas in grid
		set_obstacle(grid, position, quaternion, position_sonar, quaternion_sonar, car_range)

		# stamp current ros time to the message
		map_msg.header.stamp = rospy.Time.now()

		rospy.loginfo("Lp")

		# build ros map message and publish
		for i in range(width*height):
			map_msg.data[i] = grid.flat[i]
		occ_pub.publish(map_msg)

		rospy.loginfo("Le")

		#loop_rate.sleep()

