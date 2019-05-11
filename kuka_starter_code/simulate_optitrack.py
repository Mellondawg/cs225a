import redis
import ast

import time
import matplotlib.pyplot as plt
import math 
import numpy as np 

import argparse

#######################################################
# Constants
#######################################################

### Redis Read Keys ###
OPTITRACK_TIMESTAMP_KEY = "sai2::optitrack::timestamp"
OPTITRACK_RIGID_BODY_POSITION_KEY = "sai2::optitrack::pos_rigid_bodies"
OPTITRACK_RIGID_BODY_ORIENTATION_KEY = "sai2::optitrack::ori_rigid_bodies"
OPTITRACK_SINGLE_MARKER_POSITION_KEY = "sai2::optitrack::pos_single_markers" # targets are -2 and -3

TARGET_ROBOT_POSITION_KEY = "sai2::cs225a::kuka_robot::target::position"
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
# Preset Flight Pattern Functions
#######################################################

def figure_eight_trajectory(t):
	""" Draws a figure eight. """
	target_robot_position = [3.0, 2.0 * math.cos(t), 1.0 + 1.0 * math.cos(t) * math.sin(t)] # in robot frame
	target_optitrack_position = get_optitrack_position(target_robot_position) # in opti frame
	return target_optitrack_position, target_robot_position

def triangle_periodic_trajectory(t, period=3):
	""" Switches between the three targets. """
	first_target_position = [3.0, 1.5, 2.0]
	second_target_position = [3.0, 0.0, 1.0]
	third_target_position = [3.0, -1.5, 2.0]

	num_targets = 3
	total_period = period * num_targets
	period_t = t % total_period

	target_robot_position = first_target_position # in robot frame
	if period_t < period:
		target_robot_position = first_target_position
	elif period_t < (2 * period):
		target_robot_position = second_target_position
	else:
		target_robot_position = third_target_position

	target_optitrack_position = get_optitrack_position(target_robot_position) # in opti frame
	return target_optitrack_position, target_robot_position


#######################################################
# Parser arguments
#######################################################

parser = argparse.ArgumentParser(description='Simulate Optitrack Values in Redis.')

parser.add_argument('-p', '--preset-flight', type=str, default='figure_eight',
                    help='Choice of flight preset for the target (figure_eight, triangle_periodic). Default=figure_eight')
parser.add_argument('-t', '--time-multiplier', type=float, default=0.0005,
                    help='Multiplier applied to the time of the target speed. Default=0.0005')

args = parser.parse_args()

#######################################################
# Apply the Parser Settings
#######################################################

# Set the trajectory function to use.
target_trajectory_fn = figure_eight_trajectory
if args.preset_flight == "figure_eight":
	target_trajectory_fn = figure_eight_trajectory
else: # args.preset_flight == "triangle_periodic":
	target_trajectory_fn = triangle_periodic_trajectory

# Get the multiplier to apply to the target speed.
time_multiplier = args.time_multiplier

#######################################################
# Main
#######################################################
start_time = time.time()
niter = 0

redis_client = redis.Redis(decode_responses=True)
while True:
	# Get the current time.
	current_time = time.time() - start_time # time in seconds
	time.sleep(0.001) # delay for 1 milliseconds

	# Set the target and robot base positions.
	t = time_multiplier * niter
	
	# Get the target and robotbase positions.
	target_optitrack_position, target_robot_position = target_trajectory_fn(t)
	robotbase_optitrack_position = [0.0, 0.0, 0.0] # set optitrack and robot origins to be the same.

	# Convert to strings.
	robotbase_position_string =  " ".join(format(x, ".3f") for x in robotbase_optitrack_position)
	target_optitrack_position_string = " ".join(format(x, ".3f") for x in target_optitrack_position)
	target_robot_position_string = " ".join(format(x, ".3f") for x in target_robot_position)

	# Build the redis strings.
	rigid_body_position_value = robotbase_position_string + "; " + target_optitrack_position_string
	target_robot_position_value = target_robot_position_string

	# Set the VALUE of the rigid body KEY.
	redis_client.set(OPTITRACK_RIGID_BODY_POSITION_KEY, rigid_body_position_value)
	redis_client.set(TARGET_ROBOT_POSITION_KEY, target_robot_position_value)

	print("="*10 + " Simulated Optitrack Redis Values " + "="*10)
	print("Simluation Settings: {Preset Flight=%s, Time Multipler=%s}" % (args.preset_flight, time_multiplier))
	print("Rigid Body Positions (Optitrack Frame): ", rigid_body_position_value)
	print("Target Position (Robot Frame): ", target_robot_position_value)

	# Each iteration is ~1 milliseconds.
	niter += 1
	time_list.append(niter)