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

### Redis Write Keys ###
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

def plot_pair_plots(xs, joint_dict, title, output_filename):
	colors = ['b-', 'g-', 'r-']
	color_idx = 0
	target_joints = [0, 2, 3]
	for joint_id, joint_angles in joint_dict.items():
		if joint_id in target_joints:
			color = colors[color_idx]
			# 'b-' means blue color, round points, solid lines
			plt.plot(xs, joint_angles, color, label='JointID %s' % (joint_id+1))
			color_idx += 1

	# plt.xticks(np.arange(0,1100,100))
	# plt.yticks(np.arange(-M_PI,M_PI,0.1))
	plt.legend()

	plt.xlabel('Time (ms)')
	plt.ylabel('Joint Angle (Radans)')

	plt.tight_layout()
	plt.title(title)

	plt.savefig(output_filename, format = 'png')
	plt.close()

#######################################################
# MAIN
#######################################################
joint_dict = {} # joint_id -> values
time_list = []
controller_ran = False

counter = 0
myserver = redis.Redis(decode_responses=True)
while True:
	# Read the joint angles from redis.
	joint_angles = myserver.get(JOINT_ANGLES_KEY)
	is_running = myserver.get(CONTROLLER_RUNING_KEY)

	# Convert string literals. 
	joint_angles = ast.literal_eval(joint_angles)
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

	# Retrieve list and booleans.
	for joint_id, joint_angle in enumerate(joint_angles):
		if joint_id not in joint_dict:
			joint_dict[joint_id] = list()
		joint_dict[joint_id].append(joint_angle)

	# Each counter is 1 millisecond.
	counter += 1
	time_list.append(counter)

print(joint_dict)
print(time_list)

plot_pair_plots(time_list, joint_dict, title="Joint Angle (Radians) over Time (ms)", output_filename="q5.png")