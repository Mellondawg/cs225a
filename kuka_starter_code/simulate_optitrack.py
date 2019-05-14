import redis
import ast

import time
import math 
import numpy as np 

import argparse

#######################################################
# Constants
#######################################################

### Modes for this script. ###
READER_MODE = 0 	# only read redis values
SIMULATOR_MODE = 1 	# write AND read redis values

### Redis Read Keys ###
OPTITRACK_TIMESTAMP_KEY = "sai2::optitrack::timestamp"
OPTITRACK_RIGID_BODY_POSITION_KEY = "sai2::optitrack::pos_rigid_bodies"
OPTITRACK_RIGID_BODY_ORIENTATION_KEY = "sai2::optitrack::ori_rigid_bodies"
OPTITRACK_SINGLE_MARKER_POSITION_KEY = "sai2::optitrack::pos_single_markers" # targets are -2 and -3
CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running"

#######################################################
# Parse Arguments
#######################################################

parser = argparse.ArgumentParser(description='Simulate Optitrack Values in Redis.')

parser.add_argument('-p', '--preset-flight', type=str, default='figure_eight',
                    help='Choice of flight preset for the target (figure_eight, circle, sideways, triangle_periodic). Default=figure_eight')
parser.add_argument('-t', '--time-multiplier', type=float, default=0.05,
                    help='Multiplier applied to the time of the target speed. Default=0.05')
parser.add_argument('-d', '--disable-simulator', action='store_true',
                    help='Disable the simulator values. Only read values from redis.')

args = parser.parse_args()


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

def optitrack_to_robot_position(optitrack_position):
	""" Convert from optitrack frame to robot frame (assuming same origin). """
	opti_x, opti_y, opti_z = optitrack_position[0], optitrack_position[1], optitrack_position[2]
	robot_position = [-opti_z, -opti_x, opti_y]
	return robot_position

def robot_to_optitrack_position(robot_position):
	""" Convert from robot frame to optitrack frame (assuming same origin). """
	robo_x, robo_y, robo_z = robot_position[0], robot_position[1], robot_position[2]
	optitrack_position = [-robo_y, robo_z, -robo_x]
	return optitrack_position


#######################################################
# Preset Flight Pattern Functions
#######################################################

def figure_eight_trajectory(t):
	""" Draws a figure eight. """
	target_robot_position = [3.0, 2.0 * math.cos(math.pi * t), 1.0 + 1.0 * math.cos(math.pi * t) * math.sin(math.pi * t)] # in robot frame
	target_optitrack_position = robot_to_optitrack_position(target_robot_position) # in opti frame
	return target_optitrack_position, target_robot_position

def circle_trajectory(t, radius=0.6):
	""" Draws a circle. """ 
	target_robot_position = [3.0, radius * math.cos(math.pi * t), 1.25 + radius * math.sin(math.pi * t)] # in robot frame
	target_optitrack_position = robot_to_optitrack_position(target_robot_position) # in opti frame
	return target_optitrack_position, target_robot_position

def sideways_trajectory(t, length=4.0):
	""" Move side to side along y-axis. """
	t = float(t) / time_multiplier # ignore time multiplier.

	y = t % length  		# bound between 0 and length
	y = y - (length / 2)  	# bound between -length and +length
	if t % (2.0 * length) >= length:
		y = -1 * y
	target_robot_position = [3.0, y, 1.5]
	target_optitrack_position = robot_to_optitrack_position(target_robot_position) # in opti frame
	return target_optitrack_position, target_robot_position

def triangle_periodic_trajectory(t, period=2):
	""" Switches between the three targets. """
	t = int(t / time_multiplier) # ignore time multiplier.

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

	target_optitrack_position = robot_to_optitrack_position(target_robot_position) # in opti frame
	return target_optitrack_position, target_robot_position


#######################################################
# Read Redis Helpers
#######################################################

def parse_redis_rigid_body_matrix(rigid_body_position_value):
	robotbase_position_string, target_optitrack_position_string = rigid_body_position_value.split("; ")

	robotbase_optitrack_position = [float(x) for x in robotbase_position_string.split(" ")]
	target_optitrack_position = [float(x) for x in target_optitrack_position_string.split(" ")]

	target_robot_position = optitrack_to_robot_position(target_optitrack_position)

	return robotbase_optitrack_position, target_optitrack_position, target_robot_position


#######################################################
# Reader / Simulator Modes
#######################################################

def simulate_optitrack_redis(redis_client, niter, current_time):
	""" Simulate optitrack values and write to redis. """
	t = time_multiplier * current_time

	# Get the target and robotbase positions.
	target_optitrack_position, target_robot_position = target_trajectory_fn(t)
	robotbase_optitrack_position = [0.0, 0.0, 0.0] # set optitrack and robot origins to be the same.

	# Convert to strings.
	robotbase_position_string =  " ".join(format(x, ".3f") for x in robotbase_optitrack_position)
	target_optitrack_position_string = " ".join(format(x, ".3f") for x in target_optitrack_position)

	# Build the redis strings.
	rigid_body_position_value = robotbase_position_string + "; " + target_optitrack_position_string
	timestamp_value = "%.6f" % current_time

	# Set the VALUE of the rigid body KEY.
	redis_client.set(OPTITRACK_RIGID_BODY_POSITION_KEY, rigid_body_position_value)
	redis_client.set(OPTITRACK_TIMESTAMP_KEY, timestamp_value)

	return rigid_body_position_value #, target_robot_position_value

def read_optitrack_redis(redis_client):
	""" Log the settings and read optitrack values from redis. """
	rigid_body_position_value = redis_client.get(OPTITRACK_RIGID_BODY_POSITION_KEY)
	timestamp_value = redis_client.get(OPTITRACK_TIMESTAMP_KEY)
	
	robotbase_optitrack_position, target_optitrack_position, target_robot_position = parse_redis_rigid_body_matrix(rigid_body_position_value)
	timestamp = float(timestamp_value)

	print("="*10 + " Reading Optitrack Redis Values " + "="*10)
	if mode == SIMULATOR_MODE:
		print("SIMULATOR MODE: {Preset Flight=%s, Time Multipler=%s}" % (args.preset_flight, time_multiplier))
	print("Rigid Body Positions (Raw Redis String): \'%s\'" % rigid_body_position_value)
	print(" - Robotbase Position (Optitrack Frame): %s" % robotbase_optitrack_position)
	print(" - Target Position (Optitrack Frame):    %s" % target_optitrack_position)
	print(" - Target Position (Robot Frame):        %s" % target_robot_position)
	print("Timestamp (in seconds):          	 %s" % timestamp)

#######################################################
# Apply the Argument Settings
#######################################################

# Set the trajectory function to use.
target_trajectory_fn = figure_eight_trajectory
if args.preset_flight == "figure_eight":
	target_trajectory_fn = figure_eight_trajectory
elif args.preset_flight == "circle":
	target_trajectory_fn = circle_trajectory
elif args.preset_flight == "sideways":
	target_trajectory_fn = sideways_trajectory
else: # args.preset_flight == "triangle_periodic":
	target_trajectory_fn = triangle_periodic_trajectory

# Get the multiplier to apply to the target speed.
time_multiplier = args.time_multiplier

# Whether to only read 
mode = READER_MODE if args.disable_simulator else SIMULATOR_MODE

#######################################################
# Main
#######################################################

start_time = time.time()
niter = 0

redis_client = redis.Redis(decode_responses=True)
redis_client.set(CONTROLLER_RUNING_KEY, "1")
while True:
	# Get the current time.
	current_time = time.time() - start_time # time in seconds
	time.sleep(0.001) # delay for 1 milliseconds

	# Either write and read (simulator) or just read from redis.
	if mode == SIMULATOR_MODE:
		simulate_optitrack_redis(redis_client, niter, current_time)
		read_optitrack_redis(redis_client)
	else: # mode = READER_MODE
		read_optitrack_redis(redis_client)

	# Each iteration is ~1 milliseconds.
	niter += 1
	time_list.append(niter)