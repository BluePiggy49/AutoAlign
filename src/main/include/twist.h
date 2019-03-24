#ifndef TWIST_H
#define TWIST_H

#include "ctre/Phoenix.h"
#include "frc/WPILib.h"
#include <ctre/Phoenix.h>
#include <iostream>
#include <cmath>
#include <frc/Joystick.h>
#include <chrono>
#include <PID.h>
#include <AHRS.h>
#include <socket.h>
#include <string>
#include <cmath>

class Twist {
	public:
		Twist(TalonSRX* talon_left, TalonSRX* talon_right, PID *pid, AHRS *navx, Socket *client, frc::Joystick *joystick)
			:talon_left(talon_left), talon_right(talon_right), pid(pid), navx(navx), client(client), joystick(joystick) {
			};
			
			int auto_align();
			void wipe();
			void begin_turn(int direction_begin);
			void straight_away();
			void end_turn(int direction_turnend);
			void final_turn_go(int direction_turnend);
			int mode = 0;
			float angle_bot_turns_begin_from_jetson;
			float angle_bot_turns_end_from_jetson;
			float to_travel_distance_mm_from_jetson;
			float to_travel_distance_in_from_jetson;
			float left_depth_from_jetson;
			float right_depth_from_jetson;


	private:
		TalonSRX *talon_right, *talon_left;
		frc::Joystick *joystick;
		PID *pid;
		AHRS *navx;
		Socket *client;

		//O is path; 1 is depth wiggle
		const float enc_per_eight_feet = ((2681 + 2579) / 2.0 );
		const float encoder_counts_per_inch = enc_per_eight_feet / (8 * 12.0);

		 //center to destination distance in inches (from Jetson)
		 //angle bot should turn before it travels (from Jetson)
		float angle_bot_has_turned_begin; //Angle the bot has turned before it travels (from NavX)
		float to_travel_distance_enc; //center to destination distance in encoder counts
		float angle_bot_has_turned_end; //Angle the bot has turned after it travels (from NavX)
		 //angle bot should turn after it travels (from Jetson)
		float left_position_enc;
		float right_position_enc;

		 //(from Jetson)
		 //(from Jetson)
		float speed_ratio;
		float stop_left_depth;
		float stop_right_depth;
		float max_left_error;
		float max_right_error;
		bool buttonstate = 0;
		bool togglestate = 0;
		bool align_state = 0;
		int auto_align_mode = 0;
		int direction;
		int direction_to_turn;
		int direction_end;

		float thirty_speed = 150; //30% percent output velocity
		float twenty_speed = 126; //20% percent output velocity
		float ten_speed = 63; //10% percent output velocity

		float velocity_turn;
		float begin_yaw;

		//Possiblility for turning Speed Control
		float max_turn_velocity = 200.0;
		float min_turn_velocity = 175.0;
		float max_angle = 18.0; //Where it will start slowing
		float min_angle = 3.0; //Where we will stop
		float change_angle = 5 - min_angle;
		float change_velocity = max_turn_velocity - min_turn_velocity;
		float velocity_constant = change_velocity / change_angle;
		float d_constant = 123;
		float old_angle;
		float new_angle;
		float old_change_in_angle = 0;
		float new_change_in_angle = 0;
		float change_in_change = 0;
};
#endif