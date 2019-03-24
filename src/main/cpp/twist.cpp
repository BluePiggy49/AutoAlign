#include <twist.h>

float final_turn;

int Twist::auto_align(){
	if (mode == 0){
		std::cout<<"pig pig pig"<<std::endl;
		navx->ZeroYaw();
		
		//For when the socket is ready
		std::string socket = client->update();
		if (socket != "0")
		{
			angle_bot_turns_begin_from_jetson = client->math(1);
			to_travel_distance_mm_from_jetson = client->math(2);
			angle_bot_turns_end_from_jetson = client->math(3);
			//angle_bot_turns_begin_from_jetson = 10;
			//to_travel_distance_mm_from_jetson = 1000;
			//angle_bot_turns_end_from_jetson = -6;
			to_travel_distance_in_from_jetson = to_travel_distance_mm_from_jetson / 25.4;
			std::cout<<"Line 5"<<std::endl;
			if (angle_bot_turns_begin_from_jetson != 0 || to_travel_distance_mm_from_jetson != 0 || angle_bot_turns_end_from_jetson != 0)
			{
				mode ++;
			}
		}
	}
	if (mode == 1){ 
		//Beginning Turn Stage
		pid->PID_drivebase30();
		if (angle_bot_turns_begin_from_jetson > 0)
		{
			direction = 1;
		}else if (angle_bot_turns_begin_from_jetson < 0)
		{
			direction = -1;
		}
		begin_turn(direction);
	}
	if (mode == 2)
	{
		wipe();
		mode++;
	}
	if (mode == 3)
	{
		//Travel to Destination Point Stage
		pid->PID_drivebase30();
		straight_away();
	}
	if (mode == 4){
		navx->ZeroYaw();
		mode ++;
	}
	if (mode == 5){
		//Beginning Turn Stage
		//pid->PID_drivebase30();
		if (angle_bot_turns_end_from_jetson > 0)
		{
			direction_end = 1;
		}else if (angle_bot_turns_end_from_jetson < 0)
		{
			direction_end = -1;
		}
		end_turn(direction_end);
	}
	if (mode == 6)
	{
		navx->ZeroYaw();
		mode++;
	}
	if (mode == 9)
	{
		std::string buffer = client->update();
		std::cout<<"Left Depth: "<<client->left_depth(buffer)<<" Right Depth: "<<client->right_depth(buffer)<<" Distance: "<<client->distance_from_center(buffer)<<std::endl;
		//std::cout<<"Math 4: "<<client->math(4)<<std::endl;
		std::cout<<"Goes into atan; "<<client->distance_from_center(buffer) / (abs(client->left_depth(buffer) + client->right_depth(buffer)) /2)<<std::endl;
		final_turn = atan( client->distance_from_center(buffer) / (abs(client->left_depth(buffer) + client->right_depth(buffer)) /2.0)) * (180 / 3.1415);
		std::cout<<"Final Turn Here: "<<final_turn<<std::endl;
		if (final_turn > 0)
		{
			direction_end = 1;
			std::cout<<"Svu"<<std::endl;
		}
		else if (final_turn < 0)
		{
			direction_end = -1;
			std::cout<<"usv"<<std::endl;
		}else if (std::isnan(final_turn) == 1 )
		{
			final_turn = 0;
			std::cout<<"suv"<<std::endl;
		}
		final_turn_go(direction_end);
	}
	if (mode == 7)
	{
		std::cout<<"Yaw: "<<navx->GetYaw()<<std::endl;
	}
	std::cout<<"Yaw: "<<navx->GetYaw()<<std::endl;
	std::cout<<"Begin Yaw: "<<begin_yaw<<std::endl;
	std::cout<<"Begin Turn: "<<angle_bot_turns_begin_from_jetson<<std::endl;
	std::cout<<"Travel: "<<to_travel_distance_in_from_jetson<<std::endl;
	std::cout<<"End Turn: "<<angle_bot_turns_end_from_jetson<<std::endl;
	std::cout<<"Final Turn: "<<final_turn<<std::endl;
	//std::cout<<"Final Turn Math: "<<client->math(4)<<std::endl;
}

void Twist::wipe(){
	talon_right->SetSelectedSensorPosition(0, 0, 10);
	talon_left->SetSelectedSensorPosition(0, 0, 10);
}

void Twist::begin_turn(int direction_begin){

	std::cout<<"Direction: "<<direction_begin<<std::endl;

	angle_bot_has_turned_begin = navx->GetYaw();
	//old_angle = angle_bot_has_turned_begin;
	if (abs(abs(angle_bot_has_turned_begin) - abs(angle_bot_turns_begin_from_jetson)) <= max_angle && abs(abs(angle_bot_has_turned_begin) - abs(angle_bot_turns_begin_from_jetson)) >= min_angle){
	//If the bot's angle in comparison to the angle it should be is in between the maximum angle away and the angle it can get in between, set the speed to a calculated value.
	std::cout<<"HIIIII"<<std::endl;
	velocity_turn = velocity_constant * (abs(angle_bot_turns_begin_from_jetson) - abs(angle_bot_has_turned_begin));
	std::cout<<"Velocity of Turn: "<<velocity_turn * direction<<std::endl;	
		std::cout<<"Velocity Constant: "<<velocity_constant<<std::endl;
		
		if (velocity_turn > max_turn_velocity){
			//The Maximum Velocity is the highest velocity the robot can go
			velocity_turn = max_turn_velocity;
			std::cout<<"I'm right here"<<std::endl;
		}
		if (velocity_turn < min_turn_velocity){
			//The Miniumum Velocity is the lowest velocity the robot can go
			velocity_turn = min_turn_velocity;
			std::cout<<"Now I'm here"<<std::endl;
		}
		
		if (abs(angle_bot_has_turned_begin) < abs(angle_bot_turns_begin_from_jetson)){
			//If the bot hasn't turned enough, turn more
			talon_left->Set(ControlMode::Velocity, velocity_turn * direction); 
			talon_right->Set(ControlMode::Velocity, velocity_turn * direction); 
			std::cout<<"Whoa, I'm here now"<<std::endl;
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction<<std::endl;
		}
		else if (abs(angle_bot_has_turned_begin) > abs(angle_bot_turns_begin_from_jetson)){
			//If the bot has turned to far, turn in the other direction
			talon_left->Set(ControlMode::Velocity, -1 * velocity_turn * direction); 
			talon_right->Set(ControlMode::Velocity, -1 * velocity_turn * direction);
			std::cout<<"I'm here with a pig"<<std::endl; 
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction<<std::endl;
		}
			
		std::cout<<"Start turning"<<std::endl;
			
	}else if (abs(abs(angle_bot_has_turned_begin) - abs(angle_bot_turns_begin_from_jetson)) > max_angle){
		//If the bot's angle in comparison to the angle it should be is farther away from the angle than the maximum angle away from the angle for a reduced velocity, set the velociy to the max velocity
			if (abs(angle_bot_has_turned_begin) < abs(angle_bot_turns_begin_from_jetson)){
				//If the bot hasn't turned enough, turn more
				talon_left->Set(ControlMode::Velocity, max_turn_velocity * direction); 
				talon_right->Set(ControlMode::Velocity, max_turn_velocity * direction);

				std::cout<<"Right here right now"<<std::endl;
			}else if (abs(angle_bot_has_turned_begin) > abs(angle_bot_turns_begin_from_jetson)){
				//If the bot has turned to far, turn in the other direction
				talon_left->Set(ControlMode::Velocity, -max_turn_velocity * direction); 
				talon_right->Set(ControlMode::Velocity, -max_turn_velocity * direction); 
				std::cout<<"Down here see me"<<std::endl;
			}
			std::cout<<"More left?"<<std::endl;
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction<<std::endl;	
		
	}else if (abs(abs(angle_bot_has_turned_begin) - abs(angle_bot_turns_begin_from_jetson)) <= min_angle){
		//
		pid->PID_drivebase10();
		talon_left->Set(ControlMode::Velocity, 0); //0% of max velocity
		talon_right->Set(ControlMode::Velocity, 0);
		begin_yaw = navx->GetYaw();
		mode ++;
		std::cout<<"Dun Dun"<<std::endl;
	}
	//new_angle = navx->GetYaw();
	/*new_change_in_angle = new_angle - old_angle;
	if (old_change_in_angle != 0)
	{
		change_in_change = new_change_in_angle - old_change_in_angle;
	}*/
	//old_change_in_angle = new_change_in_angle;
	/*std::cout<<"Change in change of angle: "<<change_in_change<<std::endl;
	std::cout<<"D Value: "<<d_constant * change_in_change<<std::endl;
	std::cout<<"Yaw: "<<navx->GetYaw()<<std::endl;*/
	std::cout<<"Max Turn Velocity: "<<max_turn_velocity<<std::endl;
	std::cout<<"Velocity Constant: "<<velocity_constant<<std::endl;
	//std::cout<<"Right Position: "<<talon_right->GetSelectedSensorPosition(0)<<std::endl;
	//std::cout<<"Left Position: "<<talon_left->GetSelectedSensorPosition(0)<<std::endl;
	std::cout<<"Right Velocity: "<<talon_right->GetSelectedSensorVelocity(0)<<std::endl;
	std::cout<<"Left Velocity: "<<talon_left->GetSelectedSensorVelocity(0)<<std::endl;
	std::cout<<"Velocity of Turn: "<<velocity_turn * direction<<std::endl;
	std::cout<<"Direction Real: "<<direction<<std::endl;
}

void Twist::straight_away(){
	to_travel_distance_enc = encoder_counts_per_inch * to_travel_distance_in_from_jetson;
	talon_left->Set(ControlMode::Velocity, thirty_speed);
	talon_right->Set(ControlMode::Velocity, -thirty_speed);
	left_position_enc = talon_left->GetSelectedSensorPosition(0);
	right_position_enc = talon_right->GetSelectedSensorPosition(0);
	std::cout<<"Left Position: "<<left_position_enc<<std::endl;
	std::cout<<"Right Position: "<<right_position_enc<<std::endl;
	std::cout<<"Travel Distance: "<<to_travel_distance_enc<<std::endl;
	std::cout<<"Condition 1: "<<abs(abs(left_position_enc) - to_travel_distance_enc)<<std::endl;
	std::cout<<"Condition 2: "<<abs(abs(right_position_enc) - to_travel_distance_enc)<<std::endl;
	if ((abs(left_position_enc) >= to_travel_distance_enc) || (abs(right_position_enc) >= to_travel_distance_enc)){
		std::cout<<"Yay!"<<std::endl;
		talon_left->Set(ControlMode::Velocity, 0); //0% of max velocity
		talon_right->Set(ControlMode::Velocity, -0);
		wipe();
		mode ++;
	}
}

void Twist::end_turn(int direction_turnend){
	std::cout<<"Direction: "<<direction<<std::endl;
	angle_bot_has_turned_end = navx->GetYaw();
	//old_angle = angle_bot_has_turned_end;
	if (abs(abs(angle_bot_has_turned_end) - abs(angle_bot_turns_end_from_jetson)) <= max_angle && abs(abs(angle_bot_has_turned_end) - abs(angle_bot_turns_end_from_jetson)) >= min_angle){
	std::cout<<"HIIIII"<<std::endl;
	velocity_turn = velocity_constant * (abs(angle_bot_turns_end_from_jetson) - abs(angle_bot_has_turned_end));
	std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;	
		
		if (velocity_turn > max_turn_velocity){
			velocity_turn = max_turn_velocity;
			std::cout<<"I'm right here"<<std::endl;
		}
		if (velocity_turn < min_turn_velocity){
			velocity_turn = min_turn_velocity;
			std::cout<<"Now I'm here"<<std::endl;
		}
		
		if (abs(angle_bot_has_turned_end) < abs(angle_bot_turns_end_from_jetson)){
			talon_left->Set(ControlMode::Velocity, velocity_turn * direction_end); 
			talon_right->Set(ControlMode::Velocity, velocity_turn * direction_end); 
			std::cout<<"Whoa, I'm here now"<<std::endl;
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;
		}
		else if (abs(angle_bot_has_turned_end) > abs(angle_bot_turns_end_from_jetson)){
			talon_left->Set(ControlMode::Velocity, -velocity_turn * direction_end); 
			talon_right->Set(ControlMode::Velocity, -velocity_turn * direction_end);
			std::cout<<"I'm here with a pig"<<std::endl; 
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;
		}
			
		std::cout<<"Start turning"<<std::endl;
			
	}else if (abs(abs(angle_bot_has_turned_end) - abs(angle_bot_turns_end_from_jetson)) > max_angle){
			if (abs(angle_bot_has_turned_end) < abs(angle_bot_turns_end_from_jetson)){
				talon_left->Set(ControlMode::Velocity, max_turn_velocity * direction_end); 
				talon_right->Set(ControlMode::Velocity, max_turn_velocity * direction_end);

				std::cout<<"Right here right now"<<std::endl;
			}else if (abs(angle_bot_has_turned_end) > abs(angle_bot_turns_end_from_jetson)){
				talon_left->Set(ControlMode::Velocity, -max_turn_velocity * direction_end); 
				talon_right->Set(ControlMode::Velocity, -max_turn_velocity * direction_end); 
				std::cout<<"Down here see me"<<std::endl;
			}
			std::cout<<"More left?"<<std::endl;
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;	
		
	}else if (abs(abs(angle_bot_has_turned_end) - abs(angle_bot_turns_end_from_jetson)) <= min_angle){
		pid->PID_drivebase10();
		talon_left->Set(ControlMode::Velocity, 0); //0% of max velocity
		talon_right->Set(ControlMode::Velocity, 0);
		mode ++;
		std::cout<<"Dun Dun"<<std::endl;
	}
	/*new_angle = navx->GetYaw();
	new_change_in_angle = new_angle - old_angle;
	if (old_change_in_angle != 0)
	{
		change_in_change = new_change_in_angle - old_change_in_angle;
	}
	old_change_in_angle = new_change_in_angle;
	std::cout<<"Change in change of angle: "<<change_in_change<<std::endl;
	std::cout<<"D Value: "<<d_constant * change_in_change<<std::endl;
	std::cout<<"Yaw: "<<navx->GetYaw()<<std::endl;*/
	std::cout<<"Max Turn Velocity: "<<max_turn_velocity<<std::endl;
	//std::cout<<"Right Position: "<<talon_right->GetSelectedSensorPosition(0)<<std::endl;
	//std::cout<<"Left Position: "<<talon_left->GetSelectedSensorPosition(0)<<std::endl;
	std::cout<<"Right Velocity: "<<talon_right->GetSelectedSensorVelocity(0)<<std::endl;
	std::cout<<"Left Velocity: "<<talon_left->GetSelectedSensorVelocity(0)<<std::endl;
	std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;	
	std::cout<<"Direction End: "<<direction_end<<std::endl;
}


void Twist::final_turn_go(int direction_turnend){
	std::cout<<"Direction: "<<direction<<std::endl;
	angle_bot_has_turned_end = navx->GetYaw();
	//old_angle = angle_bot_has_turned_end;
	if (abs(abs(angle_bot_has_turned_end) - abs(final_turn)) <= max_angle && abs(abs(angle_bot_has_turned_end) - abs(final_turn)) >= min_angle){
	std::cout<<"HIIIII"<<std::endl;
	velocity_turn = velocity_constant * (abs(final_turn) - abs(angle_bot_has_turned_end));
	std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;	
		
		if (velocity_turn > max_turn_velocity){
			velocity_turn = max_turn_velocity;
			std::cout<<"I'm right here"<<std::endl;
		}
		if (velocity_turn < min_turn_velocity){
			velocity_turn = min_turn_velocity;
			std::cout<<"Now I'm here"<<std::endl;
		}
		
		if (abs(angle_bot_has_turned_end) < abs(final_turn)){
			talon_left->Set(ControlMode::Velocity, velocity_turn * direction_end); 
			talon_right->Set(ControlMode::Velocity, velocity_turn * direction_end); 
			std::cout<<"Whoa, I'm here now"<<std::endl;
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;
		}
		else if (abs(angle_bot_has_turned_end) > abs(final_turn)){
			talon_left->Set(ControlMode::Velocity, -velocity_turn * direction_end); 
			talon_right->Set(ControlMode::Velocity, -velocity_turn * direction_end);
			std::cout<<"I'm here with a pig"<<std::endl; 
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;
		}
			
		std::cout<<"Start turning"<<std::endl;
			
	}else if (abs(abs(angle_bot_has_turned_end) - abs(final_turn)) > max_angle){
			if (abs(angle_bot_has_turned_end) < abs(final_turn)){
				talon_left->Set(ControlMode::Velocity, max_turn_velocity * direction_end); 
				talon_right->Set(ControlMode::Velocity, max_turn_velocity * direction_end);

				std::cout<<"Right here right now"<<std::endl;
			}else if (abs(angle_bot_has_turned_end) > abs(final_turn)){
				talon_left->Set(ControlMode::Velocity, -max_turn_velocity * direction_end); 
				talon_right->Set(ControlMode::Velocity, -max_turn_velocity * direction_end); 
				std::cout<<"Down here see me"<<std::endl;
			}
			std::cout<<"More left?"<<std::endl;
			std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;	
		
	}else if (abs(abs(angle_bot_has_turned_end) - abs(final_turn)) <= min_angle){
		pid->PID_drivebase10();
		talon_left->Set(ControlMode::Velocity, 0); //0% of max velocity
		talon_right->Set(ControlMode::Velocity, 0);
		mode ++;
		std::cout<<"Dun Dun"<<std::endl;
	}
	/*new_angle = navx->GetYaw();
	new_change_in_angle = new_angle - old_angle;
	if (old_change_in_angle != 0)
	{
		change_in_change = new_change_in_angle - old_change_in_angle;
	}
	old_change_in_angle = new_change_in_angle;
	std::cout<<"Change in change of angle: "<<change_in_change<<std::endl;
	std::cout<<"D Value: "<<d_constant * change_in_change<<std::endl;
	std::cout<<"Yaw: "<<navx->GetYaw()<<std::endl;*/
	std::cout<<"Max Turn Velocity: "<<max_turn_velocity<<std::endl;
	//std::cout<<"Right Position: "<<talon_right->GetSelectedSensorPosition(0)<<std::endl;
	//std::cout<<"Left Position: "<<talon_left->GetSelectedSensorPosition(0)<<std::endl;
	std::cout<<"Right Velocity: "<<talon_right->GetSelectedSensorVelocity(0)<<std::endl;
	std::cout<<"Left Velocity: "<<talon_left->GetSelectedSensorVelocity(0)<<std::endl;
	std::cout<<"Velocity of Turn: "<<velocity_turn * direction_end<<std::endl;	
	std::cout<<"Direction End: "<<direction_end<<std::endl;
}