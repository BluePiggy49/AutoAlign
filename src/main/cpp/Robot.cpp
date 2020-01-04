#include "Robot.h"
#include <iostream>
#include "ctre/Phoenix.h"
#include "frc/WPILib.h"
#include <frc/Joystick.h>
#include <twist.h>
#include <PID.h>
#include <AHRS.h>
#include <stdio.h>
#include <stdlib.h>	
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <string.h>	
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <socket.h>


using namespace frc;
//using namespace std::chrono;

TalonSRX *drive_talon_right_noenc, *drive_talon_right_enc, *drive_talon_left_enc, *drive_talon_left_noenc, *claw_pivot_talon_enc, *elevator_talon_enc, *seat_motor_talon_enc, *intake_wheels_talon_noenc, *climber;
Joystick *joystick_zero, *joystick_one;
PID *pid;
Socket *client;
AHRS *navx;
Twist *twist;
//Solenoid *light;

int mode = 1;
bool pig = 0;
bool pigpig = 0;
bool dog = 0;
bool dogdog = 0;
int loop = 0;

void Robot::RobotInit() {
	/*drive_talon_left_noenc = new TalonSRX(1);
	drive_talon_left_enc = new TalonSRX(2);
	drive_talon_right_noenc = new TalonSRX(3);
	drive_talon_right_enc = new TalonSRX(4);
	drive_talon_right_noenc->Set(ControlMode::Follower, 4);
	drive_talon_left_noenc->Set(ControlMode::Follower, 2);*/
	drive_talon_left_noenc = new TalonSRX(1);
	drive_talon_left_enc = new TalonSRX(2);
	drive_talon_right_noenc = new TalonSRX(3);
	drive_talon_right_enc = new TalonSRX(4);
	//drive_talon_right_noenc->Set(ControlMode::Follower, 4);
	//drive_talon_left_noenc->Set(ControlMode::Follower, 2);
	claw_pivot_talon_enc = new TalonSRX(5);
	elevator_talon_enc = new TalonSRX(8);
	seat_motor_talon_enc = new TalonSRX(10);
	intake_wheels_talon_noenc = new TalonSRX(9);
	climber = new TalonSRX(6);

	joystick_zero = new Joystick(0);
	joystick_one = new Joystick(1);

	client = new Socket();

	pid = new PID(drive_talon_left_enc, drive_talon_right_enc, claw_pivot_talon_enc, elevator_talon_enc);

	pid->PID_drivebase30();
	pid->PID_claw_elevator();

	navx = new AHRS(SPI::Port::kMXP);

	//light = new Solenoid(1);

	twist = new Twist(drive_talon_left_enc, drive_talon_right_enc, pid, navx, client, joystick_zero);

	std::cout<<"Twist v. 56"<<std::endl;
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	drive_talon_left_enc->SetSelectedSensorPosition(0, 0, 10);
	drive_talon_right_enc->SetSelectedSensorPosition(0, 0, 10);
	navx->ZeroYaw();
}

void Robot::AutonomousPeriodic() {
	//twist->auto_align();
	//std::cout<<navx->GetYaw()<<std::endl;
	if (joystick_zero->GetRawButton(1))
	{
		climber->Set(ControlMode::PercentOutput, -0.3);
	}else if (joystick_zero->GetRawButton(2))
	{
		climber->Set(ControlMode::PercentOutput, 0.3);
	}
	else{
		climber->Set(ControlMode::PercentOutput, 0);
	}
	
	if (joystick_zero->GetRawButton(3))
	{
		elevator_talon_enc->Set(ControlMode::PercentOutput, -0.3);
	}else if (joystick_zero->GetRawButton(4))
	{
		elevator_talon_enc->Set(ControlMode::PercentOutput, 0.3);
	}
	else{
		elevator_talon_enc->Set(ControlMode::PercentOutput, 0);
	}

	if (joystick_zero->GetRawButton(5))
	{
		seat_motor_talon_enc->Set(ControlMode::PercentOutput, -0.75);
	}else if (joystick_zero->GetRawButton(6))
	{
		seat_motor_talon_enc->Set(ControlMode::PercentOutput, 0.75);
	}
	else{
		seat_motor_talon_enc->Set(ControlMode::PercentOutput, 0);
	}
	drive_talon_left_enc->Set(ControlMode::PercentOutput, 0.5 * (1 * joystick_zero->GetRawAxis(4) + (-1 * joystick_zero->GetRawAxis(1))));
	drive_talon_right_enc->Set(ControlMode::PercentOutput, 0.5 * (1 * joystick_zero->GetRawAxis(4) + (1 * joystick_zero->GetRawAxis(1))));
	//light->Set(1);
	//std::cout<<client->update()<<std::endl;
//	std::cout<<"Angle One: "<<client->math(1)<<std::endl;
//	std::cout<<"Distance: "<<client->math(2)<<std::endl;
//	std::cout<<"Angle Two: "<<client->math(3)<<std::endl;
//	std::cout<<"Math 4: "<<client->math(4)<<std::endl;
	//std::cout<<"Angle One with Filter: "<<client->median_filter(1)<<std::endl;
	//std::cout<<"Distance with Filter: "<<client->median_filter(2)<<std::endl;
	//std::cout<<"Angle Two with Filter: "<<client->median_filter(3)<<std::endl;
//	std::cout<<"Left: "<<drive_talon_left_enc->GetSelectedSensorPosition(0)<<std::endl;
//	std::cout<<"Right: "<<drive_talon_right_enc->GetSelectedSensorPosition(0)<<std::endl;
	std::cout<<"Yaw: "<<navx->GetYaw()<<std::endl;
	std::cout<<"Climber Encoder: "<<climber->GetSelectedSensorPosition(0)<<std::endl;

}

void Robot::TeleopInit() {
	navx->ZeroYaw();
	//angle_bot_turns_begin_from_jetson = -15;
	//twist->mode = 0;
	std::cout<<"Mode: "<<mode<<std::endl;
}

void Robot::TeleopPeriodic() {
	//light->Set(1);
	bool button = joystick_zero->GetRawButton(1);
	if (button == 1 && pigpig == 0 && pig == 0)
	{
		pigpig = 1;
		std::cout<<"I'm Here in Teleop"<<std::endl;
	}
	if (pigpig == 1 && joystick_zero->GetRawButton(1) == 0)
	{
		pig = 1;
		pigpig = 0;
		twist->mode = 1;
		std::cout<<"Now I'm here in teleop"<<std::endl;
	}
	if (pig == 1 && pigpig == 0)
	{
		//twist->auto_align();
		std::cout<<client->update()<<std::endl;
		//twist->mode++;
		std::cout<<"I'm aligning now"<<std::endl;
		bool button_2 = joystick_zero->GetRawButton(2);
		if (/*twist->mode == 7 || */button_2 == 1)
		{
			pig = 0;
			pigpig = 0;
			std::cout<<"Reset"<<std::endl;
			twist->mode = 0;
		}
	}
	if (button == 0 && pigpig == 0 && pig == 0)
	{
		std::cout<<"DRIVEBASE CAN RUN HERE"<<std::endl;
		climber->Set(ControlMode::PercentOutput, joystick_zero->GetRawAxis(1));
	}
	//twist->auto_align();
	//float distance = client->math(2);
	//float angle_one = client->math(1);
	//float angle_two = client->math(3);
	//std::cout<<"Twist Mode: "<<twist->mode<<std::endl;
	//std::cout<<"Angle One: "<<angle_one<<std::endl;
	//std::cout<<"Distance: "<<distance<<std::endl;
	//std::cout<<"Angle Two: "<<angle_two<<std::endl;
	//std::cout<<"Final Angle: "<<client->math(4)<<std::endl;
	//std::cout<<"Yaw: "<<navx->GetYaw()<<std::endl;
	//std::cout<<"Pigpig: "<<pigpig<<std::endl;
	//std::cout<<"Pig: "<<pig<<std::endl;
	//std::cout<<"Button: "<<button<<std::endl;
	//std::cout<<client->update()<<std::endl;
	loop++;
	std::cout<<loop<<std::endl;
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif