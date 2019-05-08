#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const std::string EE_POSITION_KEY = "sai2::cs225a::panda_robot::sensors::x";
const std::string EE_DESIRED_POSITION_KEY = "sai2::cs225a::panda_robot::sensors::xd";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string ee_link_name = "link7";
	const Vector3d ee_pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3); // task inertial matrix
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);  // null space matrix

	// x (end-effector) position values
	VectorXd ee_position_desired = VectorXd::Zero(3); 
	Vector3d ee_position = Vector3d::Zero(3); // 3d vector of zeros to fill with the end effector position
	Vector3d ee_velocity = Vector3d::Zero(3); // 3d vector of zeros to fill with the end effector velocity

	// Set q_desired
	VectorXd q_desired(7);
	q_desired = robot->_q; // initialize as initial joint angles.
	q_desired(6) = 0.1; // set last joint to 0.1 radians.

	// Set proportional and derivative gains.
	MatrixXd Kp = MatrixXd::Zero(dof,dof); // (position) p gain
	MatrixXd Kv = MatrixXd::Zero(dof,dof); // (velocity) d gain
	Kp(0, 0) = Kp(1, 1) = Kp(2, 2) = Kp(3, 3) = Kp(4, 4) = Kp(5, 5) = 400.0;
	Kp(6, 6) = 50.0;
	cout << "Kp (position) Matrix: " << endl;
	cout << Kp << endl;

	Kv(0, 0) = Kv(1, 1) = Kv(2, 2) = Kv(3, 3) = Kv(4, 4) = Kv(5, 5) = 50.0;
	Kv(6, 6) = 0.0;  // set to remove dampening.
	cout << "Kv (velocity) Matrix: " << endl;
	cout << Kv << endl;

	// Get gravity and coriolis force.
	VectorXd gravity = VectorXd::Zero(dof);
	VectorXd b = VectorXd::Zero(dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// Fill in joint gravity and coriolis and centrifugal vectors 
		robot->gravityVector(gravity);  // joint gravity vector
		robot->coriolisForce(b); // joint coriolis and centrifugal vector

		robot->position(ee_position, ee_link_name, ee_pos_in_link);
		robot->linearVelocity(ee_velocity, ee_link_name, ee_pos_in_link);

		robot->Jv(Jv, ee_link_name, ee_pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->nullspaceMatrix(N, Jv);

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			Kv(6, 6) = -0.34;  // set to remove dampening.

			command_torques = -Kp * (robot->_q - q_desired) - Kv * robot->_dq + b + gravity;			
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double m77 = robot->_M(6, 6);	  // inertia of the end-effector (m77)
			double kp = 200.0;
			double kv = 25.0; // (velocity) d gain
			Kv(0, 0) = Kv(1, 1) = Kv(2, 2) = Kv(3, 3) = Kv(4, 4) = Kv(5, 5) = Kv(6, 6) = 50.0;

			ee_position_desired << 0.3, 0.1, 0.5;

			MatrixXd F = Lambda * (kp * (ee_position_desired - ee_position) - kv * ee_velocity);
			MatrixXd damping = N.transpose() * robot->_M * Kv * robot->_dq;

			command_torques = Jv.transpose() * F + gravity - damping;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double m77 = robot->_M(6, 6);	  // inertia of the end-effector (m77)
			double kp = 200.0;
			double kv = 25.0; // (velocity) d gain
			Kv(0, 0) = Kv(1, 1) = Kv(2, 2) = Kv(3, 3) = Kv(4, 4) = Kv(5, 5) = Kv(6, 6) = 50.0;

			ee_position_desired << 0.3, 0.1, 0.5;

			MatrixXd p = J_bar.transpose() * gravity;
			MatrixXd F = Lambda * (kp * (ee_position_desired - ee_position) - kv * ee_velocity) + p;
			MatrixXd damping = N.transpose() * robot->_M * Kv * robot->_dq;

			command_torques = Jv.transpose() * F - damping;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double m77 = robot->_M(6, 6);	  // inertia of the end-effector (m77)
			double kp = 200.0;
			double kv = 25.0; // (velocity) d gain
			Kv(0, 0) = Kv(1, 1) = Kv(2, 2) = Kv(3, 3) = Kv(4, 4) = Kv(5, 5) = Kv(6, 6) = 50.0;

			ee_position_desired << 0.3 + 0.1 * sin(M_PI * time), 0.1 + 0.1 * cos(M_PI * time), 0.5;

			MatrixXd p = J_bar.transpose() * gravity;
			MatrixXd F = Lambda * (kp * (ee_position_desired - ee_position) - kv * ee_velocity) + p;

			VectorXd q_desired = VectorXd::Zero(dof); // set to all zeros
			MatrixXd damping =  N.transpose() * robot->_M * (-Kp * (robot->_q - q_desired) - Kv * robot->_dq);

			command_torques = Jv.transpose() * F + gravity + damping;

			// MatrixXd damping = N.transpose() * robot->_M * Kv * robot->_dq;
			// MatrixXd p = J_bar.transpose() * gravity;
			// Lambda = MatrixXd::Identity(3,3);
			// MatrixXd F = Lambda * (kp * (ee_position_desired - ee_position) - kv * ee_velocity) + p;

			// MatrixXd damping = N.transpose() * robot->_M * (- Kv * robot->_dq);
			// // VectorXd q_desired = VectorXd::Zero(dof); // set to all zeros
			// // MatrixXd damping =  N.transpose() * robot->_M * (-Kp * (robot->_q - q_desired) - Kv * robot->_dq + b);

			// command_torques = Jv.transpose() * F + damping;
		}

		// End after 10 second.
		if(time >= 10.0) {
			break;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixJSON(EE_POSITION_KEY, ee_position);
		redis_client.setEigenMatrixJSON(EE_DESIRED_POSITION_KEY, ee_position_desired);

		controller_counter++;
	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
