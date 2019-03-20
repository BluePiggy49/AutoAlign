#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	float max_turn_velocity = 400;
	float min_turn_velocity = 100;
	float max_angle = 30; //Where it will start slowing
	float min_angle = 1; //Where we will stop
	float change_angle = max_angle - min_angle;
	float change_velocity = max_turn_velocity - min_turn_velocity;
	float velocity_constant = change_velocity / change_angle;
	float angle_bot_turns_begin_from_jetson;
	float angle_bot_has_turned_begin;
};
