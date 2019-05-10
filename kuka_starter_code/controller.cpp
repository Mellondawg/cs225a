// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "chai3d.h"

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

//------------------------------------------------------------------------------
// REDIS KEYS
//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------
// CONTROLLER SETTINGS 
//------------------------------------------------------------------------------

const bool flag_simulation = true;
const bool use_optitrack = false; // whether to use optitrack values from redis.
const bool inertia_regularization = true;

//------------------------------------------------------------------------------
// HELPER FUNCTIONS
//------------------------------------------------------------------------------

/**
 * Converts the target position (in optitrack frame) and robot base position (in optitrack frame) 
 * into the target position in the robot frame.
 */
Vector3d getTargetRobotPosition(Vector3d target_optitrack_position, 
								Vector3d baseframe_optitrack_position) {
	Vector3d target_robot_position = Vector3d::Zero();
	Vector3d relative_optitrack_position = Vector3d::Zero();

	relative_optitrack_position = (target_optitrack_position - baseframe_optitrack_position);

	// Baseframe will give a translation.
	double relative_optitrack_x =  relative_optitrack_position(0);
	double relative_optitrack_y =  relative_optitrack_position(1);
	double relative_optitrack_z =  relative_optitrack_position(2);

	// Rotate optitrack axes into robot axes.
	target_robot_position << -relative_optitrack_z, -relative_optitrack_x, relative_optitrack_y;
	return target_robot_position;
}

/**
 * Retrieves a preset target position (in robot frame) based on the 
 * cumulative time run and period time. 
 */
Vector3d getPeriodicPosition(double time,  // cumulative time
							 int period) { // time spent on each target

	Vector3d target_robot_position = Vector3d::Zero();

	// drone position in base frame
	Vector3d target1_position = Vector3d(3.0, 1.5, 2.0); 
	Vector3d target2_position = Vector3d(3.0, 0.0, 1.0); 
	Vector3d target3_position = Vector3d(3.0, -1.5, 2.0);

	// set the number of targets and periodic time
	int num_targets = 3;
	int periodic_time = int(time) % (num_targets * period);

	// switch between the 3 targets
	if (periodic_time < period) {
		target_robot_position = target1_position;
	} else if (periodic_time < 2 * period) {
		target_robot_position = target2_position;
	} else {
		target_robot_position = target3_position;
	}

	return target_robot_position;
}

/**
 * Gets the desired orientation (in robot frame) from the target position (in robot frame)
 * and the end-effector position (in robot frame) by transforming a direction vector into
 * an orientation matrix.
 */
Matrix3d getDesiredOrientation(Vector3d target_robot_position, // target (in robot frame)
							   Vector3d ee_robot_position, 	   // end-effector (in robot frame)
							   bool verbose=false) {   

	Matrix3d desired_orientation = Matrix3d::Identity();

	// get the unit direction vector
	Vector3d direction_vector = (target_robot_position - ee_robot_position);
	direction_vector.normalize();

	// set the desired orientation matrix			
	Quaterniond qrotation;
	qrotation.setFromTwoVectors(Vector3d::UnitZ(), direction_vector);
	desired_orientation = qrotation.toRotationMatrix();

	// debugging statements
	if (verbose) {
		cout << "Current and Target Positions: " << endl;
		cout << ee_robot_position << endl; 
		cout << target_robot_position << endl; 

		cout << "Desired Orientation: " << endl;
		cout << desired_orientation << endl;
	}
	
	return desired_orientation;
}

//------------------------------------------------------------------------------
// MAIN
//------------------------------------------------------------------------------

int main() {	

	//------------------------------------------------------------------------------
	// COMMUNICATION SETUP
	//------------------------------------------------------------------------------

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
		OPTITRACK_SINGLE_MARKER_POSITION_KEY = "sai2::optitrack::pos_single_markers"; // targets are -2 and -3
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	//------------------------------------------------------------------------------
	// ROBOT CONTROLLER SETUP
	//------------------------------------------------------------------------------

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	//------------------------------------------------------------------------------
	// OPTITRAK TASK
	//------------------------------------------------------------------------------

	// optitrack frame
	Vector3d baseframe_optitrack_position = Vector3d::Zero(); // robot base position (optitrack frame)
	Vector3d target_optitrack_position = Vector3d::Zero();    // drone position (optitrack frame)
	MatrixXd optitrack_rigid_positions = MatrixXd::Zero(2,3); 

	//------------------------------------------------------------------------------
	// DRONE TASK
	//------------------------------------------------------------------------------

	// drone (robot frame)
	Vector3d target_robot_position = Vector3d::Zero();

	//------------------------------------------------------------------------------
	// POSE TASK
	//------------------------------------------------------------------------------

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

	//------------------------------------------------------------------------------
	// JOINT TASK
	//------------------------------------------------------------------------------

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
	q_init_desired << 70.0, -30.0, -15.0, 60.0, 30.0, -90.0, 0.0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	//------------------------------------------------------------------------------
	// CONTROL LOOP
	//------------------------------------------------------------------------------

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	if (use_optitrack) {
		// read the robot base position and target position from optitrack
		redis_client.getEigenMatrixDerivedString(OPTITRACK_RIGID_BODY_POSITION_KEY,optitrack_rigid_positions);
		baseframe_optitrack_position = optitrack_rigid_positions.row(0); // first rigid position
		target_optitrack_position = optitrack_rigid_positions.row(1); // second rigid position
	}

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		
		if (use_optitrack) {
			// update the optitrack target position
			redis_client.getEigenMatrixDerivedString(OPTITRACK_RIGID_BODY_POSITION_KEY,optitrack_rigid_positions);
			target_optitrack_position = optitrack_rigid_positions.row(1); // second rigid position
		}

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
				robot->_M(4,4) += 0.15;
				robot->_M(5,5) += 0.15;
				robot->_M(6,6) += 0.15;
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

			// choose a target position
			if (use_optitrack) 
			{
				target_robot_position = getTargetRobotPosition(target_optitrack_position, baseframe_optitrack_position); 
			} else {
				int period = 2; // how long to spend at a specific target
				target_robot_position = getPeriodicPosition(time, period);
			}

			// set the desired orientation of the pose task
			posori_task->_desired_orientation = getDesiredOrientation(target_robot_position, posori_task->_current_position);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
