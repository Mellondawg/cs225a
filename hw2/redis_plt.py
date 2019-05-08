import redis
import time
import ast
import matplotlib.pyplot as plt

#######################################################
# Constants
#######################################################

### Redis Read Keys ###
JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q"
JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq"
EE_POSITION_KEY = "sai2::cs225a::panda_robot::sensors::x"
EE_DESIRED_POSITION_KEY = "sai2::cs225a::panda_robot::sensors::xd"
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

def plot_joint_pair_plots(xs, joint_dict, title, output_filename):
	color_idx = 0
	target_joints = [0, 1, 2, 3, 4, 5, 6] # plot all the joints
	for joint_id, joint_angles in joint_dict.items():
		if joint_id in target_joints:
			color = colors[color_idx]
			# 'b-' means blue color, round points, solid lines
			plt.plot(xs, joint_angles, color, label='JointID %s' % (joint_id+1))
			color_idx = (color_idx + 1) % len(colors)

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
time_list = []

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

	joint_angles = ast.literal_eval(joint_angles)
	ee_position = ast.literal_eval(ee_position)
	ee_desired_position = ast.literal_eval(ee_desired_position)

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


plot_joint_pair_plots(time_list, joint_dict, title="Joint Angle (Radians) over Time (ms)", output_filename="q4aiv-joints.png")
plot_desired_position_pair_plots(time_list, ee_position_dict, ee_desired_position_dict, title="End-Effector Position over Time (ms)", output_filename="q4aiv-position.png")

