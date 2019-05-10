import redis
import ast

import time
import matplotlib.pyplot as plt
import math 
import numpy as np 

#######################################################
# Constants
#######################################################

### Redis Read Keys ###
OPTITRACK_TIMESTAMP_KEY = "sai2::optitrack::timestamp"
OPTITRACK_RIGID_BODY_POSITION_KEY = "sai2::optitrack::pos_rigid_bodies"
OPTITRACK_RIGID_BODY_ORIENTATION_KEY = "sai2::optitrack::ori_rigid_bodies"
OPTITRACK_SINGLE_MARKER_POSITION_KEY = "sai2::optitrack::pos_single_markers" # targets are -2 and -3

JOINT_ANGLES_KEY = "sai2::cs225a::kuka_robot::sensors::q"

#######################################################
# Stored Values
#######################################################

joint_dict = {} # joint_id -> values
ee_position_dict = {} # 'x', 'y', 'z' -> values
ee_position_dict['x'] = list()
ee_position_dict['y'] = list()
ee_position_dict['z'] = list()
ee_desired_position_dict = {} # 'x', 'y', 'z' -> values
ee_desired_position_dict['x'] = list()
ee_desired_position_dict['y'] = list()
ee_desired_position_dict['z'] = list()
ee_velocity_dict = {} # 'x', 'y', 'z' -> values
ee_velocity_dict['dx'] = list()
ee_velocity_dict['dy'] = list()
ee_velocity_dict['dz'] = list()
time_list = []

#######################################################
# Conversion between Optitrack / Robot Frames
#######################################################

def get_robot_position(optitrack_position):
	""" Convert from optitrack frame to robot frame (assuming same origin). """
	opti_x, opti_y, opti_z = optitrack_position[0], optitrack_position[1], optitrack_position[2]
	robot_position = [-opti_z, -opti_x, opti_y]
	return robot_position

def get_optitrack_position(robot_position):
	""" Convert from robot frame to optitrack frame (assuming same origin). """
	robo_x, robo_y, robo_z = robot_position[0], robot_position[1], robot_position[2]
	optitrack_position = [-robo_y, robo_z, -robo_x]
	return optitrack_position


#######################################################
# Preset Flight Patterns
#######################################################

def figure_eight_trajectory(t):
	target_robot_position = [3.0, 2.0 * math.cos(t), 1.0 + 1.0 * math.cos(t) * math.sin(t)] # in robot frame
	target_optitrack_position = get_optitrack_position(target_robot_position) # in opti frame
	return target_optitrack_position


#######################################################
# MAIN
#######################################################

print("Waiting for controller to begin...")

start_time = time.time()
niter = 0

redis_client = redis.Redis(decode_responses=True)
while True:
	# Get the current time.
	current_time = time.time() - start_time # time in seconds
	time.sleep(0.001) # delay for 1 milliseconds

	# Set the target and robot base positions.
	t = 0.001 * niter
	
	# Get the target and robotbase positions.
	target_optitrack_position = figure_eight_trajectory(t)
	robotbase_optitrack_position = [0.0, 0.0, 0.0] # set optitrack and robot origins to be the same.

	# Convert to strings.
	robotbase_position_string =  " ".join(format(x, ".3f") for x in robotbase_optitrack_position)
	target_position_string = " ".join(format(x, ".3f") for x in target_optitrack_position)

	# Set the VALUE of the rigid body KEY.
	rigid_body_position_value = robotbase_position_string + "; " + target_position_string
	redis_client.set(OPTITRACK_RIGID_BODY_POSITION_KEY, rigid_body_position_value)

	print("Optitrack Rigid Body Positions: ", rigid_body_position_value)

	# Each iteration is ~1 milliseconds.
	niter += 1
	time_list.append(niter)