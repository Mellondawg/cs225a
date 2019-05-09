// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/kuka_iiwa.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

int state = JOINT_CONTROLLER;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// - optitrack
std::string OPTITRACK_TIMESTAMP_KEY;
std::string OPTITRACK_RIGID_BODY_POSITION_KEY;
std::string OPTITRACK_RIGID_BODY_ORIENTATION_KEY;
std::string OPTITRACK_SINGLE_MARKER_POSITION_KEY;

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool use_optitrack = false;

const bool inertia_regularization = true;

Vector3d getCalibratedPosition(Vector3d optitrack_target_position, Vector3d optitrack_baseframe_position) {
	Vector3d robot_calibrated_position = Vector3d::Zero();
	Vector3d optitrack_relative_position = Vector3d::Zero();

	optitrack_relative_position = (optitrack_target_position - optitrack_baseframe_position);

	// Baseframe will give a translation.
	double opti_relative_x =  optitrack_relative_position(0);
	double opti_relative_y =  optitrack_relative_position(1);
	double opti_relative_z =  optitrack_relative_position(2);

	// Rotate optitrack axes into robot axes.

	robot_calibrated_position << -opti_relative_z, -opti_relative_x, opti_relative_y;
	return robot_calibrated_position;
}

int main() {	

	// 1) How do move visual objects from the URDF file.

	/** COMMUNICATION SETUP **/

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY = "sai2::cs225a::kuka_robot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::cs225a::kuka_robot::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::kuka_robot::actuators::fgc";

		OPTITRACK_TIMESTAMP_KEY = "sai2::optitrack::timestamp";
		OPTITRACK_RIGID_BODY_POSITION_KEY = "sai2::optitrack::pos_rigid_bodies";
		OPTITRACK_RIGID_BODY_ORIENTATION_KEY = "sai2::optitrack::ori_rigid_bodies";
		OPTITRACK_SINGLE_MARKER_POSITION_KEY = "sai2::optitrack::pos_single_markers"; // targets are -2 and -3
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";	

		OPTITRACK_TIMESTAMP_KEY = "sai2::optitrack::timestamp";
		OPTITRACK_RIGID_BODY_POSITION_KEY = "sai2::optitrack::pos_rigid_bodies";
		OPTITRACK_RIGID_BODY_ORIENTATION_KEY = "sai2::optitrack::ori_rigid_bodies";
		OPTITRACK_SINGLE_MARKER_POSITION_KEY = "sai2::optitrack::pos_single_markers";	
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	/** ROBOT CONTROLLER SETUP **/

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	/** DRONE TASK **/

	// drone optitrack
	Vector3d optitrack_target_position = Vector3d::Zero();
	Vector3d optitrack_baseframe_position = Vector3d::Zero();

	// drone position in base frame
	Vector3d target1_position = Vector3d(3.0, 1.5, 2.0); 
	Vector3d target2_position = Vector3d(3.0, 0.0, 1.0); 
	Vector3d target3_position = Vector3d(3.0, -1.5, 2.0);

	/** POSE TASK **/

	// pose task
	const string control_link = "link6";
	const Vector3d control_point = Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	// use online trjaectory generation
#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif
	
	// set the gains on the pose task
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	/** JOINT TASK **/

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	// set the gains on the joint task
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 15.0;

	// set the initial (default) desired position of the joint task
	VectorXd q_init_desired = initial_q;
	q_init_desired << 120.0, -30.0, -15.0, 60.0, 30.0, -90.0, 0.0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	/** CONTROLLER LOOP **/

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// read the values from optitrack.
	MatrixXd optitrack_rigid_positions = redis_client.getEigenMatrixJSON(OPTITRACK_RIGID_BODY_POSITION_KEY);
	cout << optitrack_rigid_positions << endl;
	// optitrack_target_position = optitrack_rigid_positions(0);
	// optitrack_baseframe_position = optitrack_rigid_positions(1);

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model (simulation or kinematics)
		if(flag_simulation)
		{
			robot->updateModel();
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();
		}

		if(state == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			// check whether actual joint position is close to desired joint position
			if( (robot->_q - q_init_desired).norm() < 0.05 )
			{
				posori_task->reInitializeTask();
				posori_task->_desired_position = Vector3d(0.0,0.0,0.75);

				// TODO: Point in direction of x-axis initially.
				posori_task->_desired_orientation = AngleAxisd(0.0, Vector3d::UnitX()).toRotationMatrix(); //* posori_task->_desired_orientation;

				joint_task->reInitializeTask();
				joint_task->_kp = 0;

				state = POSORI_CONTROLLER;
			}
		}

		else if(state == POSORI_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// switch between the targets (5 seconds each)
			Vector3d chosen_target_position = getCalibratedPosition(optitrack_target_position, optitrack_baseframe_position); 
			// int periodic_time = int(time) % 15;
			// if (periodic_time % 15 < 5) {
			// 	chosen_target_position = target1_position;
			// } else if (periodic_time % 15 < 10) {
			// 	chosen_target_position = target2_position;
			// } else {
			// 	chosen_target_position = target3_position;
			// }

			// get the unit direction vector
			Vector3d direction_vector = (chosen_target_position - posori_task->_current_position);
			direction_vector.normalize();

			// set the desired orientation matrix			
			Quaterniond qrotation;
			qrotation.setFromTwoVectors(Vector3d::UnitZ(), direction_vector);
			posori_task->_desired_orientation = qrotation.toRotationMatrix();

			cout << "Current and Target Positions: " << endl;
			cout << posori_task->_current_position << endl; 
			cout << chosen_target_position << endl; 

			cout << "Desired Orientation: " << endl;
			cout << posori_task->_desired_orientation << endl;

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
