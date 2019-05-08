import redis
import time
import ast
import matplotlib.pyplot as plt

### Redis Read Keys ###
JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q"
JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq"

### Redis Write Keys ###
JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc"
CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running"


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
