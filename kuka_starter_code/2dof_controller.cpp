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
#include <cmath>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/kuka_iiwa.urdf";

#define JOINT_CONTROLLER              0
#define TASK_CONTROLLER               1

#define OPTITRACK_RIGIDBODY_ROBOTBASE_INDEX       0    // index for the robot base rigid body 
#define OPTITRACK_RIGIDBODY_TARGET_INDEX          1    // index for the target rigid body 

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

std::string CONTROLLER_RUNING_KEY;

//------------------------------------------------------------------------------
// CONTROLLER SETTINGS 
//------------------------------------------------------------------------------

const bool flag_simulation = true;
const bool use_optitrack = true; // whether to use optitrack values from redis.
const bool inertia_regularization = true;

unsigned long long controller_counter = 0;

//------------------------------------------------------------------------------
// TARGETING FUNCTIONS
//------------------------------------------------------------------------------

/**
 * Gets the target position (in the robot frame) from the target position (in optitrack frame) 
 * and robot base position (in optitrack frame).
 */
Vector3d getTargetRobotPosition(Vector3d target_optitrack_position, 
								Vector3d robotbase_optitrack_position) {
	Vector3d target_robot_position = Vector3d::Zero();
	Vector3d relative_optitrack_position = Vector3d::Zero();

	// get the vector from the robot base to the target position
	relative_optitrack_position = (target_optitrack_position - robotbase_optitrack_position);

	// extract the xyz coordinates
	double relative_optitrack_x = relative_optitrack_position(0);
	double relative_optitrack_y = relative_optitrack_position(1);
	double relative_optitrack_z = relative_optitrack_position(2);

	// rotate optitrack axes into robot axes.
	target_robot_position << -relative_optitrack_z, -relative_optitrack_x, relative_optitrack_y;
	return target_robot_position;
}

/**
 * Gets an estimate of the target velocity from the current and previous positions and 
 * change in time, i.e. dx = (x_{t} - x_{t-1}) / dt
 */
Vector3d estimateTargetRobotVelocity(Vector3d current_target_position, 
									 Vector3d previous_target_position,
									 double dt) {
	Vector3d estimate_target_velocity = Vector3d::Zero();
	if (dt != 0.0) {
		estimate_target_velocity = (current_target_position - previous_target_position) / dt;
	}
	return estimate_target_velocity;
}

/**
 * Gets an estimate of the target velocity from the current and previous positions and 
 * change in time, i.e. dx = (x_{t} - x_{t-1}) / dt
 */
Vector3d estimateTargetRobotAcceleration(Vector3d current_target_velocity, 
									 	 Vector3d previous_target_velocity,
									 	 double dt) {
	Vector3d estimate_target_acceleration = Vector3d::Zero();
	if (dt != 0.0) {
		estimate_target_acceleration = (current_target_velocity - previous_target_velocity) / dt;
	}
	return estimate_target_acceleration;
}

/**
 * Predicts the future position of the target position, 
 * i.e. x = x_prev + (dx * dt)
 */
Vector3d predictedTargetRobotPosition(Vector3d current_position, 
									  Vector3d estimate_velocity, 
									  Vector3d estimate_acceleration,
									  double dt) {
	Vector3d predicted_target_position = Vector3d::Zero();
	predicted_target_position = current_position + estimate_velocity * dt + (0.5) * estimate_acceleration * pow(dt, 2); 
	return predicted_target_position;
}

/**
 * Gets the desired orientation (robot frame) from the target position (robot frame)
 * and the end-effector position (robot frame) by transforming a direction vector into
 * an orientation matrix.
 */
Matrix3d getDesiredOrientation(Vector3d target_robot_position, // target position (robot frame)
							   Vector3d ee_robot_position,     // end-effector position (robot frame)
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
	}
	else
	{
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
		
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";
	}
	
	OPTITRACK_TIMESTAMP_KEY = "sai2::optitrack::timestamp";
	OPTITRACK_RIGID_BODY_POSITION_KEY = "sai2::optitrack::pos_rigid_bodies";
	OPTITRACK_RIGID_BODY_ORIENTATION_KEY = "sai2::optitrack::ori_rigid_bodies";
	OPTITRACK_SINGLE_MARKER_POSITION_KEY = "sai2::optitrack::pos_single_markers"; // targets are -2 and -3

	CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

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
	VectorXd gravity = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint and torque limits
	const double torque_limit[7] = {165, 165, 100, 100, 100, 35, 35}; // LBR iiwa 7 R800
	const double joint_limit[7] = {165.0/180.0*M_PI, 115.0/180.0*M_PI, 165.0/180.0*M_PI, 115.0/180.0*M_PI, 165.0/180.0*M_PI, 119.0/180.0*M_PI, 173.0/180.0*M_PI};

	//------------------------------------------------------------------------------
	// OPTITRAK TASK
	//------------------------------------------------------------------------------

	// raw optitrack data
	MatrixXd optitrack_rigid_positions = MatrixXd::Zero(2,3); 

	// optitrack positions
	Vector3d robotbase_optitrack_position = Vector3d::Zero(); // robot base position (optitrack frame)
	Vector3d target_optitrack_position = Vector3d::Zero();    // target position (optitrack frame)

	//------------------------------------------------------------------------------
	// DRONE TASK
	//------------------------------------------------------------------------------

	// target 3d coordinates (robot frame)
	Vector3d target_robot_position = Vector3d::Zero();
	Vector3d target_robot_velocity = Vector3d::Zero();

	// target 3d coordinates (ee frame)
	Vector3d target_ee_position = Vector3d::Zero();

	// target 2dof coordinates (ee frame)
	Vector2d target_2dof_position = Vector2d::Zero();
	Vector2d target_2dof_position_desired = Vector2d::Zero();
	Vector2d target_2dof_velocity = Vector2d::Zero();

	//------------------------------------------------------------------------------
	// DRONE TASK
	//------------------------------------------------------------------------------

	// pose task
	const string control_link = "link6";
	const Vector3d control_point = Vector3d(0,0,0.07);
	Vector3d ee_robot_position = Vector3d::Zero();

	// task jacobian
	Eigen::MatrixXd J0 = MatrixXd::Zero(6,dof);

	// rotation matrices
	Eigen::Matrix3d R0_ee = Matrix3d::Zero();
	Eigen::MatrixXd R = MatrixXd::Zero(6,6);

	// transformation matrices
	Eigen::Affine3d T0_ee = Affine3d::Identity(); // transform from base to end-effector
	Eigen::MatrixXd H = MatrixXd::Zero(2,6);	  

	// controller matrices
	Eigen::MatrixXd Jtilde = MatrixXd::Zero(2,dof);
	Eigen::MatrixXd Jbar = MatrixXd::Zero(dof,2);  // Note: Only Sai2Model uses internally.
	Eigen::MatrixXd Lambda = MatrixXd::Zero(2,2); 
	Eigen::MatrixXd N = MatrixXd::Zero(dof,dof); 

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
	q_init_desired << 90.0, -90.0, 0.0, -90.0, 0.0, 0.0, 0.0;
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

	// store times read from optitrack
	double optitrack_time = 0.0; 	  //secs
	double optitrack_prev_time = 0.0; //secs

	if (use_optitrack) {
		// read the robot base position from the optitrack
		redis_client.getEigenMatrixDerivedString(OPTITRACK_RIGID_BODY_POSITION_KEY, optitrack_rigid_positions);
		robotbase_optitrack_position = optitrack_rigid_positions.row(OPTITRACK_RIGIDBODY_ROBOTBASE_INDEX);

		// read the timestamp from the optitrack
		string optitrack_timestamp_string = redis_client.get(OPTITRACK_TIMESTAMP_KEY);
		// TODO: Convert Optitrack timestamp (string) to seconds (double).
		optitrack_time = optitrack_prev_time = std::stod(optitrack_timestamp_string);
	}

	redis_client.set(CONTROLLER_RUNING_KEY, "0");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		
		if (use_optitrack) {
			// update the target position from the optitrack
			redis_client.getEigenMatrixDerivedString(OPTITRACK_RIGID_BODY_POSITION_KEY, optitrack_rigid_positions);
			target_optitrack_position = optitrack_rigid_positions.row(OPTITRACK_RIGIDBODY_TARGET_INDEX);

			// convert optitrack timestamp (string) to seconds (double)
			string optitrack_timestamp_string = redis_client.get(OPTITRACK_TIMESTAMP_KEY);
			optitrack_time = std::stod(optitrack_timestamp_string);
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
			//if( (robot->_q - q_init_desired).norm() < 0.05 )
			string running_string = redis_client.get(CONTROLLER_RUNING_KEY);
			if ( running_string == "1" )
			{
				joint_task->reInitializeTask();
				joint_task->_kp = 0;

				state = TASK_CONTROLLER;
			}
		}

		else if(state == TASK_CONTROLLER)
		{
			// update robot values
			robot->position(ee_robot_position, control_link, control_point);
			robot->transform(T0_ee, control_link, control_point);
			robot->rotation(R0_ee, control_link);
			robot->gravityVector(gravity);  // joint gravity vector
			robot->J(J0, control_link, control_point);

			// choose a target position
			Vector3d current_target_robot_position = getTargetRobotPosition(target_optitrack_position, robotbase_optitrack_position); 
			double delta_time = optitrack_time - optitrack_prev_time;

			// only update if the optitrack time has changed
			if ( delta_time != 0.0 ) {

				// get the estimated velocity and acceleration from position and time data.
				Vector3d estimate_target_robot_velocity = estimateTargetRobotVelocity(current_target_robot_position, target_robot_position, delta_time);
				Vector3d estimate_target_robot_acceleration = estimateTargetRobotAcceleration(estimate_target_robot_velocity, target_robot_velocity, delta_time);

				// predict target position in a timestep
				Vector3d predicted_target_robot_position = predictedTargetRobotPosition(current_target_robot_position, estimate_target_robot_velocity, estimate_target_robot_acceleration, delta_time);

				// set the desired orientation of the pose task
				// posori_task->_desired_orientation = getDesiredOrientation(predicted_target_robot_position, ee_robot_position);

				// update target position and (estimated) velocity
				target_robot_position = current_target_robot_position;
				target_robot_velocity = estimate_target_robot_velocity;
				
				// update the time
				optitrack_prev_time = optitrack_time;
			} 

			// ------- CONTROLLER EQNS -------

			// compute R
			MatrixXd zero_3d = MatrixXd::Zero(3,3);
			R << R0_ee.transpose(), zero_3d, // [[R0_ee, 0]
				 zero_3d, R0_ee.transpose(); //  [0, R0_ee]]

			// compute target coordinates in ee_frame
			target_ee_position = T0_ee.inverse() * target_robot_position; 
			double target_ee_x = target_ee_position(0);
			double target_ee_y = target_ee_position(1);
			double target_ee_z = target_ee_position(2);
			//cout << T0_ee.linear().inverse() * (ee_robot_position - T0_ee.translation()) << endl;  	// should equal (0, 0, 0)
			//cout << target_ee_position << endl;  	// should equal (0, 0, 0)

			// compute H
			H << 1, 0, 0, 0, -target_ee_z, target_ee_y,
			     0, 0, 1, -target_ee_y, target_ee_x, 0;

			// compute Jtilde
			Jtilde = H * R * J0;  // (2 x dof) = (2, 6) x (6, 6) x (6 x dof)

			// compute Lambda, Jbar, N from Jtilde
			robot->operationalSpaceMatrices(Lambda, Jbar, N, Jtilde);

			// set the gains
			double kp = 50.0;
			double kv = 25.0;
			// double kpj = 100.0;
			// double kvj = 50.0;

			// set the target actual and desired position
			target_2dof_position << target_ee_x, target_ee_z;
			target_2dof_position_desired << 0.0, 0.0;
			target_2dof_velocity = Jtilde * robot->_dq;

			// ==> F = (2 x 2) * (2 x 1) = (2 x 1)
			MatrixXd F = Lambda * (-kp * (target_2dof_position - target_2dof_position_desired) - kv * target_2dof_velocity);

			// ==> damping = (dof x dof) x (dof x 1) = (dof x 1)
			// MatrixXd damping = robot->_M * (-kpj * (robot->_q - q_init_desired) - kvj * robot->_dq);


			joint_task->updateTaskModel(N);
			joint_task->computeTorques(joint_task_torques);

			// ==> torques = (dof x 2) x (2 x 1) + (dof x dof) x (dof x 1) = (dof x 1)
			command_torques = Jtilde.transpose() * F + joint_task_torques; // N.transpose() * damping;
		}

		// detect joint and torque limits
		// command_torques << 170, 170, 170, -170, -50, -100, -110;
		// robot->_q  << 180.0/180.0*M_PI, 140.0/180.0*M_PI, -170.0/180.0*M_PI, 100.0/180.0*M_PI, 165.0/180.0*M_PI, 120.0/180.0*M_PI, -180.0/180.0*M_PI;
		for (int i = 0; i < dof; i++) {
			double torque_i = command_torques(i);
			if (abs(torque_i) > torque_limit[i]) {
				cout << "[WARNING] Joint: " << i << endl;
				cout << "[WARNING] Torque (Actual): " << torque_i << endl;
				cout << "[WARNING] Torque (Limit): " << torque_limit[i] << endl;
			}

			double robot_qi = robot->_q(i);
			if (abs(robot_qi) > joint_limit[i]) {
				cout << "[ERROR] Joint: " << i << endl;
				cout << "[ERROR] Angle (Actual): " << robot_qi << endl;
				cout << "[ERROR] Angle (Limit): " << joint_limit[i] << endl;	
			}
		}

		// send torques to redis.
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// update control loop counter
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
