#define PI 3.14159
#define RADIANSTODEGREES 180/PI
#define DEGREESTORADIANS PI/180
#include <socket.h>
//#include <ioctl.h>

std::string Socket::update() {
  std::string angle = "A";
  std::string left = "L";
  std::string right = "R";
  if (mode == 1)
	{
		//std::cout<<"I am at line 55"<<std::endl;
		my_socket = socket(AF_INET, SOCK_STREAM, 0);
		memset(&serv_addr, '0', sizeof(serv_addr));
		memset(&my_address, '0', sizeof(my_address));
		my_address.sin_family = AF_INET;
		my_address.sin_addr.s_addr = inet_addr("10.9.55.193");
		my_address.sin_port = htons(5060);
		//my_socket = my_socket;
		if (my_socket < 0){
			std::cout<<"Failed to Start"<<std::endl;
			return "0";
		}else{
			mode ++;
			//connect_mode++;
		}
	}

	connect_socket = connect(my_socket, (struct sockaddr *)&my_address, sizeof(my_address));
	///std::cout<<"Connect Socket: "<<connect_socket<<std::endl;
	if (connect_socket < 0 && connect_mode == 1) {
		std::cout<<"Could Not Connect"<<std::endl;
		connect_mode = 1;
		//mode = 1;
		//return "Could Not Connect dangit";
		return "0";
	}else{
		connect_mode ++;
		flags = fcntl(my_socket, F_GETFL);
		fcntl(my_socket, F_SETFL, flags | O_NONBLOCK);
		//std::cout<<"Flags: "<<flags<<std::endl;
		//flags = ioctl(my_socket, FIONBIO);
		bzero(buffer, sizeof(buffer));
		int reading = read(my_socket, buffer, 100);
		//std::cout<<"Size of Buffer: "<<sizeof(buffer)<<std::endl;
		//std::cout<<"What is reading? "<<reading<<std::endl;
		if (reading > 0)
		{
			buffer_string = "";
			for (std::size_t char_buffer_place = 0; char_buffer_place < reading; char_buffer_place++)
			{
				char current_buffer_char = buffer[char_buffer_place];
				std::string current_buffer_string(1, current_buffer_char);
				buffer_string = buffer_string + current_buffer_char;
				//std::cout<<"Buffer String: "<<buffer_string<<std::endl;
				//std::cout<<"Here"<<std::endl;
				//std::string search_string = buffer_string;
			}
		//	std::cout<<"Angle Location: "<<buffer_string.find(angle)<<std::endl;
		//	std::cout<<"Left Location: "<<buffer_string.find(left)<<std::endl;
		//	std::cout<<"Right Location: "<<buffer_string.find(right)<<std::endl;
		if (buffer_string.find(angle) == 0 && buffer_string.find(left) >= 0 && buffer_string.find(right) >= 0)
		{
			//std::cout<<"Gonna print it out"<<std::endl;
			//std::cout<<"This is the buffer_string: "<<buffer_string<<std::endl;
			return buffer_string;
		}
			//std::cout<<"raw buffer: "<<buffer<<std::endl;
			
		}else {
			std::cout<<"Not Receiving Message"<<std::endl;
			//connect_mode = 1;
			//mode = 1;
			//return "Reading " + std::to_string(reading) + " buffer is here " + buffer;
			return "0";
		}
	}
	//connect_socket = -1;
	//return "Connect mode " + std::to_string(connect_mode) + " mode " + std::to_string(mode) + " Bytes read " + std::to_string(reading) + " This is the buffer " + buffer;
	//std::cout<<"Loop: "<<loop<<std::endl;
	loop++;
}

/*float Socket::angle_one(std::string buffer_string){
	std::string angle = "A";
	std::string dest = "B";
	std::string angle_string = "";
	std::string dest_string = "";

	std::size_t find_angle_begin = buffer_string.find(angle);
	std::size_t find_angle_length = angle.find_last_of(angle);
	std::size_t find_angle_end = find_angle_begin + find_angle_length; //adds the length of the string to where it started
	std::size_t find_angle_number_start = find_angle_end + 2;


	std::size_t find_dest_begin = buffer_string.find(dest);

	std::size_t find_angle_number_end = find_dest_begin - 2;


	for (std::size_t current_angle_char_num = find_angle_number_start; current_angle_char_num <= find_angle_number_end; current_angle_char_num++){
		char current_angle_char = buffer_string[current_angle_char_num];
		std::string current_angle_string(1, current_angle_char);
		angle_string = angle_string + current_angle_char;
	}

	float angle_float = atof(angle_string.c_str());

	//std::cout<<"Angle Float: "<<angle_float<<std::endl;

	return angle_float;
}

float Socket::distance(std::string buffer_string){
	std::string dest = "B";
	std::string newang = "C";

	std::string dest_string = "";
	std::string newang_string = "";

	std::size_t find_dest_begin = buffer_string.find(dest);
	std::size_t find_dest_length = dest.find_last_of(dest);
	std::size_t find_dest_end = find_dest_begin + find_dest_length; //adds the length of the string to where it started
	std::size_t find_dest_number_start = find_dest_end + 2;

	std::size_t find_newang_begin = buffer_string.find(newang);

	std::size_t find_dest_number_end = find_newang_begin - 2;

	for (std::size_t current_dest_char_num = find_dest_number_start; current_dest_char_num <= find_dest_number_end; current_dest_char_num++){
		char current_dest_char = buffer_string[current_dest_char_num];
		std::string current_dest_string(1, current_dest_char);
		dest_string = dest_string + current_dest_char;
	}

	float dest_float = atof(dest_string.c_str());

	//std::cout<<"Destination Float: "<<dest_float<<std::endl;

	return dest_float;
}

float Socket::angle_two(std::string buffer_string){
	std::string newang = "C";
	std::string left = "L";

	std::string newang_string = "";
	std::string left_string = "";

	std::size_t find_newang_begin = buffer_string.find(newang);
	std::size_t find_newang_end = buffer_string.find_last_of(newang);
	std::size_t find_newang_number_start = find_newang_end + 2;

	std::size_t find_left_begin = buffer_string.find(left);

	std::size_t find_newang_number_end = find_left_begin - 2;
	
	for (std::size_t current_newang_char_num = find_newang_number_start; current_newang_char_num <= find_newang_number_end; current_newang_char_num++){
		char current_newang_char = buffer_string[current_newang_char_num];
		std::string current_newang_string(1, current_newang_char);
		newang_string = newang_string + current_newang_char;
	}

	float newang_float = atof(newang_string.c_str());

	//std::cout<<"New Angle Float: "<<newang_float<<std::endl;

	return newang_float;
}
*/

float Socket::distance_from_center(std::string buffer_string){
	std::string angle = "A";
	std::string dest = "L";
	std::string angle_string = "";
	std::string dest_string = "";

	std::size_t find_angle_begin = buffer_string.find(angle);
	std::size_t find_angle_length = angle.find_last_of(angle);
	std::size_t find_angle_end = find_angle_begin + find_angle_length; //adds the length of the string to where it started
	std::size_t find_angle_number_start = find_angle_end + 2;


	std::size_t find_dest_begin = buffer_string.find(dest);

	std::size_t find_angle_number_end = find_dest_begin - 2;


	for (std::size_t current_angle_char_num = find_angle_number_start; current_angle_char_num <= find_angle_number_end; current_angle_char_num++){
		char current_angle_char = buffer_string[current_angle_char_num];
		std::string current_angle_string(1, current_angle_char);
		angle_string = angle_string + current_angle_char;
	}

	float angle_float = atof(angle_string.c_str());

	//std::cout<<"Angle Float: "<<angle_float<<std::endl;
	//buffer_string = "";
	return angle_float;
}

float Socket::left_depth(std::string buffer_string){
	std::string left = "L";
	std::string right = "R";

	std::string left_string = "";
	std::string right_string = "";

	std::size_t find_left_begin = buffer_string.find(left);
	std::size_t find_left_length = left.find_last_of(left);
	std::size_t find_left_end = find_left_begin + find_left_length;
	std::size_t find_left_number_start = find_left_end + 2;

	std::size_t find_right_begin = buffer_string.find(right);

	std::size_t find_left_number_end = find_right_begin - 2;

	for (std::size_t current_left_char_num = find_left_number_start; current_left_char_num <= find_left_number_end; current_left_char_num++){
		char current_left_char = buffer_string[current_left_char_num];
		std::string current_left_string(1, current_left_char);
		left_string = left_string + current_left_char;
		//std::cout<<"left_string: "<<left_string<<std::endl;
	}
	
	float left_float = atof(left_string.c_str());

	//std::cout<<"Left Depth Float: "<<left_float<<std::endl;
	//buffer_string = "";
	return left_float;
}

float Socket::right_depth(std::string buffer_string){
	std::string right = "R";

	std::string right_string = "";

	std::size_t find_right_begin = buffer_string.find(right);
	std::size_t find_right_length = right.find_last_of(right);
	std::size_t find_right_end = find_right_begin + find_right_length;
	std::size_t find_right_number_start = find_right_end + 2;

	std::size_t find_right_number_end = buffer_string.find_last_of(buffer_string);

	for (std::size_t current_right_char_num = find_right_number_start; current_right_char_num <= find_right_number_end; current_right_char_num++){
		char current_right_char = buffer_string[current_right_char_num];
		std::string current_right_string(1, current_right_char);
		right_string = right_string + current_right_char;
	//	std::cout<<"right_string: "<<right_string<<std::endl;
	}

	float right_float = atof(right_string.c_str());

	//std::cout<<"Right Depth Float: "<<right_float<<std::endl;
	//buffer_string = "";
	return right_float;
}

float Socket::math(int version){
	if (update() != "0"){
		float LeftDepth = left_depth(update());
		float RightDepth = right_depth(update());
		//std::cout << "Right Depth: " << RightDepth << std::endl;
		//std::cout << "Left Depth: " << LeftDepth << std::endl;
		float VisionTargetDistanceFromCenter = distance_from_center(update());
		if (LeftDepth != 0 || RightDepth != 0 || VisionTargetDistanceFromCenter != 0)
		{
			if (LeftDepth == 0 || RightDepth == 0)
			{
				return 0;
			}
			//std::cout<<"Distance from Center: "<<VisionTargetDistanceFromCenter<<std::endl;
			//int ControlPointDistanceFromVisionTarget = 1000;
			float DepthDifference = (RightDepth - LeftDepth);
			float CenterDepth = abs(LeftDepth + RightDepth) / 2; 
			float VisionTargetLength = 430;

			//Vision Target Slope is negative if the right depth is closer than the left depth and vice versa ******** Works just like slope 
			float VisionTargetSlope = RADIANSTODEGREES * asin((DepthDifference) / VisionTargetLength);
			float ControlPointDistanceFromCenter;
			float ControlPointXDistanceFromVisionTarget;
			float ControlPointYDistanceFromVisionTarget;
			float CameraYDistanceFromControlPoint;
			float CameraXDistanceFromControlPoint;
			float FirstAngleToMove;
			float DistanceToMove;
			float SecondAngleToMove;

			//std::cout << "Angle of Target " << VisionTargetSlope << std::endl;

			//Calculating the control point's position to the vision target
			ControlPointXDistanceFromVisionTarget = 1000 * (sin(DEGREESTORADIANS * VisionTargetSlope));
			ControlPointYDistanceFromVisionTarget = abs(1000 * (cos(DEGREESTORADIANS * VisionTargetSlope)));

			//std::cout << "CP X/Y Distance from VT " << ControlPointXDistanceFromVisionTarget << " " << ControlPointYDistanceFromVisionTarget << std::endl; 

			//Calculating the camera's position to the control point
			CameraYDistanceFromControlPoint = CenterDepth - ControlPointYDistanceFromVisionTarget;
			CameraXDistanceFromControlPoint = VisionTargetDistanceFromCenter + ControlPointXDistanceFromVisionTarget;

			//Calculating the angles and stuff
			FirstAngleToMove = RADIANSTODEGREES * atan(CameraXDistanceFromControlPoint/CameraYDistanceFromControlPoint);
			DistanceToMove = sqrt(pow(CameraXDistanceFromControlPoint, 2) + pow(CameraYDistanceFromControlPoint, 2));
			SecondAngleToMove = -1 * (FirstAngleToMove + VisionTargetSlope);
			//std::cout<<"VisionTargetDistanceFromCenter: "<<VisionTargetDistanceFromCenter<<std::endl;
			//std::cout<<"CenterDepth: "<<CenterDepth<<std::endl;
			//std::cout<<"Atan Calculation: "<<(VisionTargetDistanceFromCenter / CenterDepth)<<std::endl;

			if (version == 1){
				//std::cout<<"FirstAngleToMove: "<<FirstAngleToMove<<std::endl;
				return FirstAngleToMove;
			}
			if (version == 2){
				return DistanceToMove;
			}
			if (version == 3){
				return SecondAngleToMove;
			}
			if (version == 4)
			{
				return (atan(VisionTargetDistanceFromCenter / CenterDepth) * RADIANSTODEGREES);
			}
		}else{
			return 0;
		}
	}else{
		return 0;
	}
}

float Socket::median_filter(int math_version){
	if (median_level == 1){
		box_1 = math(math_version);
		median_level++;
	}
	if (median_level == 2)
	{
		box_2 = math(math_version);
		if (box_1 > box_2)
		{
			save_box = box_2;
			box_2 = box_1;
			box_1 = save_box;
		}
		median_level++;
	}
	if (median_level == 3){
		box_3 = math(math_version);
		if (box_2 > box_3)
		{
			save_box = box_3;
			box_3 = box_2;
			box_2 = save_box;
		}
		if (box_1 > box_2)
		{
			save_box = box_2;
			box_2 = box_1;
			box_1 = save_box;
		}
		median_level++;
	}
	
	if (median_level == 4)
	{
		box_4 = math(math_version);
		if (box_3 > box_4)
		{
			save_box = box_4;
			box_4 = box_3;
			box_3 = save_box;
		}
		if (box_2 > box_3)
		{
			save_box = box_3;
			box_3 = box_2;
			box_2 = save_box;
		}
		if (box_1 > box_2)
		{
			save_box = box_2;
			box_2 = box_1;
			box_1 = save_box;
		}
		median_level++;
	}
	
	if (median_level == 5)
	{
		box_5 = math(math_version);
		if (box_4 > box_5)
		{
			save_box = box_5;
			box_5 = box_4;
			box_4 = save_box;
		}
		if (box_3 > box_4)
		{
			save_box = box_4;
			box_4 = box_3;
			box_3 = save_box;
		}
		if (box_2 > box_3)
		{
			save_box = box_3;
			box_3 = box_2;
			box_2 = save_box;
		}
		if (box_1 > box_2)
		{
			save_box = box_2;
			box_2 = box_1;
			box_1 = save_box;
		}
		median_level++;
	}
	return box_3;
}