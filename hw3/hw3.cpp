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
const std::string EE_VELOCITY_KEY = "sai2::cs225a::panda_robot::sensors::dx";
const std::string EE_DESIRED_POSITION_KEY = "sai2::cs225a::panda_robot::sensors::xd";
const std::string EE_DELTA_ANGLE_KEY = "sai2::cs225a::panda_robot::sensors::dphi";
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
	const Vector3d ee_pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// prepare end-effector values
	VectorXd ee_position_desired = VectorXd::Zero(3); 
	VectorXd ee_velocity_desired = VectorXd::Zero(3);
	VectorXd ee_acceleration_desired = VectorXd::Zero(3);
	Matrix3d ee_rotation_desired = Matrix3d::Zero();

	Vector3d ee_position = Vector3d::Zero(3); // 3d vector of zeros to fill with the end effector position
	Vector3d ee_velocity = Vector3d::Zero(3); // 3d vector of zeros to fill with the end effector velocity
	Matrix3d ee_rotation = Matrix3d::Zero();

	Vector3d ee_angular_velocity = Vector3d::Zero(3);

	// Set q_desired
	VectorXd q_desired = VectorXd::Zero(dof);
	VectorXd q_upper = VectorXd::Zero(dof);
	VectorXd q_lower = VectorXd::Zero(dof);
	VectorXd gravity = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Jw = MatrixXd::Zero(3,dof);
	MatrixXd J_0 = MatrixXd::Zero(6,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd Lambda_0 = MatrixXd::Zero(6,6);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

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

		robot->Jv(Jv, ee_link_name, ee_pos_in_link);
		robot->Jw(Jw, ee_link_name);
		robot->J_0(J_0, ee_link_name, ee_pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->taskInertiaMatrix(Lambda_0, J_0);
		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->nullspaceMatrix(N, Jv);

		robot->gravityVector(gravity);  // joint gravity vector
		robot->position(ee_position, ee_link_name, ee_pos_in_link);
		robot->linearVelocity(ee_velocity, ee_link_name, ee_pos_in_link);
		robot->rotation(ee_rotation, ee_link_name);
		robot->angularVelocity(ee_angular_velocity, ee_link_name);

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;  
		
		VectorXd delta_angle = Vector3d::Zero(3);

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			// set the gains
			double kp = 100.0; 
			double kv = 20.0;
			double kpj = 50.0;
			double kvj = 14.0;

			ee_position_desired << 0.3 + 0.1 * sin(M_PI * time), 0.1 + 0.1 * cos(M_PI * time), 0.5;
			ee_velocity_desired << 0.1 * M_PI * cos(M_PI * time), -0.1 * M_PI * sin(M_PI * time), 0.0;
			ee_acceleration_desired << -0.1 * M_PI * M_PI * sin(M_PI * time), -0.1 * M_PI * M_PI * cos(M_PI * time), 0.0;

			MatrixXd F = Lambda * (ee_acceleration_desired - kp * (ee_position - ee_position_desired) - kv * (ee_velocity - ee_velocity_desired));
			MatrixXd damping =  N.transpose() * (-kpj * (robot->_q - q_desired) - kvj * robot->_dq);

			command_torques = Jv.transpose() * F + damping + gravity;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			// set the gains
			double kp = 100.0; 
			double kv = 20.0;
			double kdamp = 7.0;
			double kmid = 25.0;
			
			// set the joint limits
			q_upper << -165, -100, -165, -170, -165, 0, -165; 
			q_lower << 165, 100, 165, -30, 165, 210, 165; 
			q_upper = q_upper * (M_PI / 180);
			q_lower = q_lower * (M_PI / 180);
			VectorXd q_mid = (q_upper + q_lower) / 2.0;

			ee_position_desired << -0.65, -0.45, 0.7; // -0.1, 0.15, 0.2;

			VectorXd gamma_mid = -2.0 * kmid * (robot->_q - q_mid);
			VectorXd gamma_damp = -kdamp * robot->_dq;
			MatrixXd F = Lambda * (-kp * (ee_position - ee_position_desired) - kv * ee_velocity);

			command_torques = Jv.transpose() * F + N.transpose() * gamma_mid + N.transpose() * gamma_damp + gravity;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			// set the gains
			double kp = 50.0; 
			double kv = 20.0;
			double kpj = 50.0;
			double kvj = 14.0;

			// Set desired position and orientation.
			ee_position_desired << 0.6, 0.3, 0.5;

			ee_rotation_desired(0, 0) = cos(M_PI/3);
			ee_rotation_desired(0, 1) = 0;
			ee_rotation_desired(0, 2) = sin(M_PI/3);

			ee_rotation_desired(1, 0) = 0;
			ee_rotation_desired(1, 1) = 1;
			ee_rotation_desired(1, 2) = 0;

			ee_rotation_desired(2, 0) = -sin(M_PI/3);
			ee_rotation_desired(2, 1) = 0;
			ee_rotation_desired(2, 2) = cos(M_PI/3);

			// Compute the change in rotation.
			for (int i = 0; i < 3; i++) {
				delta_angle += ee_rotation.col(i).cross(ee_rotation_desired.col(i));
			}
			delta_angle = -0.5 * delta_angle;

			MatrixXd f0 = kp * (ee_position_desired - ee_position) - kv * ee_velocity; // (3x1)
			MatrixXd f1 = kp * (-delta_angle) - kv * ee_angular_velocity; // (3x1)
			MatrixXd F(6, 1);
			F << f0, // stacked vertically (readibility).
				 f1;
			F = Lambda_0 * F; // (6x6) x (6x1) = (6x1), and J_0.T is (7x6), so term is (7x1).

			MatrixXd damping = -N.transpose() * kvj * robot->_dq;

			command_torques = J_0.transpose() * F + damping + gravity;
		} 

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			// set the gains
			double kp = 200.0;
			double kv = 25.0;
			double kpj = 50.0;
			double kvj = 14.0;

			// set the desired position
			ee_position_desired << 0.6, 0.3, 0.4;

			// set the controller values
			ee_velocity_desired = (kp / kv) * (ee_position_desired - ee_position);

			double Vmax = 0.1; // maximum velocity (m/s)
			double x = Vmax / ee_velocity_desired.norm();
			double v = 0.0; // implement saturation function
			if (abs(x) <= 1.0) {
				v = x; 
			} else{
				v = signbit(x);
			}

			MatrixXd F = Lambda * (-kv * (ee_velocity - v * ee_velocity_desired));
			MatrixXd damping =  N.transpose() * robot->_M * (-kpj * (robot->_q - q_desired) - kvj * robot->_dq);

			command_torques = Jv.transpose() * F + damping + gravity;
		}

		// End after t seconds.
		if(time >= 10.0) {
			break;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixJSON(EE_POSITION_KEY, ee_position);
		redis_client.setEigenMatrixJSON(EE_VELOCITY_KEY, ee_velocity);
		redis_client.setEigenMatrixJSON(EE_DESIRED_POSITION_KEY, ee_position_desired);
		redis_client.setEigenMatrixJSON(EE_DELTA_ANGLE_KEY, delta_angle);

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
