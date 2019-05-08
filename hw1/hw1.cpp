#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4
#define QUESTION_5   5

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_controller.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
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
	VectorXd command_torques = VectorXd::Zero(dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// Pull out estimates
	bool printed_values = false;
	double m = robot->_M(0, 0);	 // inertia of the joint (m11)
	VectorXd gravity = VectorXd::Zero(dof);
	VectorXd coriolisForce = VectorXd::Zero(dof);
	VectorXd q_desired = VectorXd::Zero(dof); 

	std::cout << "\n";
	std::cout << "Initial Joint Positions  : " << initial_q << endl;
	std::cout << "Initial Estimate of Inertia (m11) : " << m << endl; 

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_1;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5

		// Set the desired robot joint angles.
		q_desired << 90, -45, 0, -125, 0, 80, 0;
		q_desired *= (M_PI / 180);

		// Fill in joint gravity and coriolis and centrifugal vectors 
		robot->gravityVector(gravity);  // joint gravity vector
		robot->coriolisForce(coriolisForce); // joint coriolis and centrifugal vector

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0;      			// (position) p gain
			double kv = 2.0 * sqrt(kp * m);     // (velocity) d gain
			if(!printed_values) {
				cout << "Control Position Gain (p) : " << kp << endl; 
				cout << "Control Velocity Gain (d) : " << kv << endl; 
				cout << endl;
				printed_values = true;
			}

			// Compute command torques.
			command_torques = -kp * (robot->_q - q_desired) - kv * robot->_dq;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 400.0;      			// (position) p gain
			double kv = 2.0 * sqrt(kp * m);     // (velocity) d gain
			cout << "Control Position Gain (p) : " << kp << endl; 
			cout << "Control Velocity Gain (d) : " << kv << endl; 
			cout << "Gravity (g) : " << endl; 
			cout << gravity << endl;
			cout << endl;

			command_torques = -kp * (robot->_q - q_desired) - kv * robot->_dq + gravity;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 400.0;      			// (position) p gain
			double kv = 2.0 * sqrt(kp * m);     // (velocity) d gain
			cout << "Control Position Gain (p) : " << kp << endl; 
			cout << "Control Velocity Gain (d) : " << kv << endl; 
			cout << "Mass Matrix  A(q): " << endl;
			cout << robot->_M << endl;
			cout << "Gravity (g) : " << endl; 
			cout << gravity << endl;
			cout << endl;

			command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + gravity;		
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double kp = 400.0;      			// (position) p gain
			double kv = 2.0 * sqrt(kp * m);     // (velocity) d gain
			cout << "Control Position Gain (p) : " << kp << endl; 
			cout << "Control Velocity Gain (d) : " << kv << endl; 
			cout << "Mass Matrix  A(q): " << endl;
			cout << robot->_M << endl;
			cout << "Coriolis Force (b) : " << endl; 
			cout << coriolisForce << endl;
			cout << "Gravity (g) : " << endl; 
			cout << gravity << endl;
			cout << endl;

			command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + coriolisForce + gravity;		
		}

		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_5)
		{
			double kp = 400.0;      			// (position) p gain
			double kv = 2.0 * sqrt(kp * m);     // (velocity) d gain
			cout << "Control Position Gain (p) : " << kp << endl; 
			cout << "Control Velocity Gain (d) : " << kv << endl; 
			cout << "Mass Matrix  A(q): " << endl;
			cout << robot->_M << endl;
			cout << "Coriolis Force (b) : " << endl; 
			cout << coriolisForce << endl;
			cout << "Gravity (g) : " << endl; 
			cout << gravity << endl;
			cout << endl;

			command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + coriolisForce + gravity;			
		}

		// End after 1 second.
		if(time >= 1.0) {
			break;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

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
