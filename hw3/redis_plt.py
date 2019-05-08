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
JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q"
JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq"
EE_POSITION_KEY = "sai2::cs225a::panda_robot::sensors::x"
EE_VELOCITY_KEY = "sai2::cs225a::panda_robot::sensors::dx"
EE_DESIRED_POSITION_KEY = "sai2::cs225a::panda_robot::sensors::xd"
EE_DELTA_ANGLE_KEY = "sai2::cs225a::panda_robot::sensors::dphi"
JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc"
CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running"

#######################################################
# Parameters
#######################################################

# Amount of time to run simulation
NUM_MS_IN_SECOND = 1000
NUM_SECONDS = 4

#######################################################
# PLOTS
#######################################################
colors = ['b-', 'g-', 'r-', 'y-', 'c-', 'm-', 'k-']
upper_limit_colors = ['b-.', 'g-.', 'r-.']
lower_limit_colors = ['b--', 'g--', 'r--']

def plot_joint_pair_plots(xs, joint_dict, title, output_filename):
	color_idx = 0
	target_joints = [3, 5] # plot all the joints (4 and 6)
	for joint_id, joint_angles in joint_dict.items():
		if joint_id in target_joints:
			color = colors[color_idx]
			# 'b-' means blue color, round points, solid lines
			plt.plot(xs, joint_angles, color, label='JointID %s' % (joint_id+1))
			color_idx = (color_idx + 1) % len(colors)

	# Plot Joint 4 Limits.
	lower_color = lower_limit_colors[0]
	upper_color = upper_limit_colors[0]
	joint_angles = [-170 * (math.pi / 180)] * len(xs)
	plt.plot(xs, joint_angles, lower_color, label='JointID %s Lower Limit' % (4))
	joint_angles = [-30 * (math.pi / 180)] * len(xs)
	plt.plot(xs, joint_angles, upper_color, label='JointID %s Upper Limit' % (4))

	# Plot Joint 6 Limits.
	lower_color = lower_limit_colors[1]
	upper_color = upper_limit_colors[1]
	joint_angles = [0 * (math.pi / 180)] * len(xs)
	plt.plot(xs, joint_angles, lower_color, label='JointID %s Lower Limit' % (6))
	joint_angles = [210 * (math.pi / 180)] * len(xs)
	plt.plot(xs, joint_angles, upper_color, label='JointID %s Upper Limit' % (6))

	plt.legend()

	plt.xlabel('Time (ms)')
	plt.ylabel('Joint Angle (Radians)')

	plt.title(title)

	plt.savefig(output_filename, format = 'png')
	plt.close()

def plot_position_pair_plots(xs, position_dict, title, output_filename):
	colors = ['b-', 'g-', 'r-']
	color_idx = 0
	for axis_label, position_values in position_dict.items():
		color = colors[color_idx]
		# 'b-' means blue color, round points, solid lines
		plt.plot(xs, position_values, color, label='Axis: %s' % axis_label)
		color_idx = (color_idx + 1) % len(colors)

	plt.legend()

	plt.xlabel('Time (ms)')
	plt.ylabel('Position (Meters)')

	plt.title(title)

	plt.savefig(output_filename, format = 'png')
	plt.close()

def plot_velocity_pair_plots(xs, velocity_dict, title, output_filename):
	colors = ['b--', 'g--', 'r--']
	color_idx = 0
	for axis_label, velocity_values in velocity_dict.items():
		color = colors[color_idx]
		# 'b-' means blue color, round points, solid lines
		plt.plot(xs, velocity_values, color, label='Axis: %s' % axis_label)
		color_idx = (color_idx + 1) % len(colors)

	V_max = [0.1] * len(xs)
	plt.plot(xs, V_max, 'c-', label='V_max')

	velocity_norm = []
	dx = velocity_dict['dx']
	dy = velocity_dict['dy']
	dz = velocity_dict['dz']
	for dx, dy, dz in zip(dx, dy, dz):
		v_norm = math.sqrt(dx**2 + dy**2 + dz**2)
		velocity_norm.append(v_norm)
	plt.plot(xs, velocity_norm, 'm-', label='End-Effector Velocity Norm')

	plt.legend()

	plt.xlabel('Time (ms)')
	plt.ylabel('Velocity (m/s)')

	plt.title(title)

	plt.savefig(output_filename, format = 'png')
	plt.close()

def plot_desired_position_pair_plots(xs, position_dict, position_desired_dict, title, output_filename):
	colors = ['b-', 'g-', 'r-']
	d_colors = ['b--', 'g--', 'r--']
	color_idx = 0
	for axis_label, position_values in position_dict.items():
		desired_position_values = position_desired_dict[axis_label]
		color = colors[color_idx]
		d_color = d_colors[color_idx]
		# 'b-' means blue color, round points, solid lines
		plt.plot(xs, position_values, color, label='Actual %s' % axis_label)
		plt.plot(xs, desired_position_values, d_color, label='Desired %s' % axis_label)
		color_idx = (color_idx + 1) % len(colors)

	plt.legend()

	plt.xlabel('Time (ms)')
	plt.ylabel('Position (Meters)')

	plt.title(title)

	plt.savefig(output_filename, format = 'png')
	plt.close()

def plot_dphi_pair_plots(xs, dphi0_vals, dphi1_vals, dphi2_vals, title, output_filename):
	colors = ['b-', 'g-', 'r-']

	plt.plot(xs, dphi0_vals, colors[0], label='dphi_0')
	plt.plot(xs, dphi1_vals, colors[1], label='dphi_1')
	plt.plot(xs, dphi2_vals, colors[2], label='dphi_2')

	plt.legend()

	plt.xlabel('Time (ms)')
	plt.ylabel('Orientation Error (Radians)')

	plt.title(title)

	plt.savefig(output_filename, format = 'png')
	plt.close()


#######################################################
# MAIN
#######################################################

print("Waiting for controller to begin...")

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

dphi_0 = []
dphi_1 = []
dphi_2 = []

controller_ran = False

counter = 0
myserver = redis.Redis(decode_responses=True)
while True:
	# Convert string literals. 
	is_running = myserver.get(CONTROLLER_RUNING_KEY)
	is_running = ast.literal_eval(is_running)
	is_running = bool(is_running) # convert from int to boolean.

	time.sleep(0.001) # delay for 1 millisecond
	# Start when controller is running.
	if is_running:
		controller_ran = True
	# End if not running and controller already ran once.
	else:
		if controller_ran == True:
			break
		# Otherwise controller has not yet run, keep waiting.
		continue

	# Read the joint angles from redis.
	joint_angles = myserver.get(JOINT_ANGLES_KEY)
	ee_position = myserver.get(EE_POSITION_KEY)
	ee_desired_position = myserver.get(EE_DESIRED_POSITION_KEY)
	ee_velocity = myserver.get(EE_VELOCITY_KEY)
	ee_delta_angle = myserver.get(EE_DELTA_ANGLE_KEY)

	joint_angles = ast.literal_eval(joint_angles)
	ee_position = ast.literal_eval(ee_position)
	ee_desired_position = ast.literal_eval(ee_desired_position)
	ee_velocity = ast.literal_eval(ee_velocity)
	ee_delta_angle = ast.literal_eval(ee_delta_angle)

	# Retrieve list and booleans.
	for joint_id, joint_angle in enumerate(joint_angles):
		if joint_id not in joint_dict:
			joint_dict[joint_id] = list()
		joint_dict[joint_id].append(joint_angle)

	x, y, z = ee_position[0], ee_position[1], ee_position[2]
	ee_position_dict['x'].append(x)
	ee_position_dict['y'].append(y)
	ee_position_dict['z'].append(z)

	xd, yd, zd = ee_desired_position[0], ee_desired_position[1], ee_desired_position[2]
	ee_desired_position_dict['x'].append(xd)
	ee_desired_position_dict['y'].append(yd)
	ee_desired_position_dict['z'].append(zd)

	dx, dy, dz = ee_velocity[0], ee_velocity[1], ee_velocity[2]
	ee_velocity_dict['dx'].append(dx)
	ee_velocity_dict['dy'].append(dy)
	ee_velocity_dict['dz'].append(dz)

	dphi0, dphi1, dphi2 = ee_delta_angle[0], ee_delta_angle[1], ee_delta_angle[2]
	dphi_0.append(dphi0)
	dphi_1.append(dphi1)
	dphi_2.append(dphi2)

	# Each counter is 1 millisecond.
	counter += 1
	time_list.append(counter)

print(joint_dict)
print(ee_position_dict)
print(ee_desired_position_dict)
print(time_list)

# plot_joint_pair_plots(time_list, joint_dict, title="Joint Angle (Radians) over Time (ms)", output_filename="q1e.png")

# plot_position_pair_plots(time_list, ee_position_dict, title="End-Effector Position over Time (ms)", output_filename="q3a-position.png")
# plot_joint_pair_plots(time_list, joint_dict, title="Joint Angle (Radians) over Time (ms)", output_filename="q3a-joints.png")

# plot_joint_pair_plots(time_list, joint_dict, title="Joint Angle (Radians) over Time (ms)", output_filename="q2f-joints.png")
# plot_desired_position_pair_plots(time_list, ee_position_dict, ee_desired_position_dict, title="End-Effector Position over Time (ms)", output_filename="q2f-position.png")

# plot_desired_position_pair_plots(time_list, ee_position_dict, ee_desired_position_dict, title="End-Effector Position over Time (ms)", output_filename="q3a-position.png")
# plot_dphi_pair_plots(time_list, dphi_0, dphi_1, dphi_2, title="Orientation Error over Time (ms)", output_filename="q3a-orientation.png")

plot_desired_position_pair_plots(time_list, ee_position_dict, ee_desired_position_dict, title="End-Effector Position over Time (ms)", output_filename="q4b-position.png")
plot_velocity_pair_plots(time_list, ee_velocity_dict, title="End-Effector Velocity over Time (ms)", output_filename="q4b-velocity.png")

