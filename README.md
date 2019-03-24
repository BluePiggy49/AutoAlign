For Integration into Robot Code:


```
bool button = joystick_zero->GetRawButton(1);
	if (button == 1 && pigpig == 0 && pig == 0)
	{
		pigpig = 1;
	}
	if (pigpig == 1 && joystick_zero->GetRawButton(1) == 0)
	{
		pig = 1;
		pigpig = 0;
		twist->mode = 1;
	}
	if (pig == 1 && pigpig == 0)
	{
		twist->auto_align();
		std::cout<<"I'm aligning now"<<std::endl;
		bool button_2 = joystick_zero->GetRawButton(2);
		if (twist->mode == 7)
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
	}
```

If you want to be able to stop it manually, add a toggle function to 


```
if (twist->mode == 7) //Add a toggle into this if statement. If a button is pressed for the second time, it will stop toggling.
		{
			pig = 0;
			pigpig = 0;
			std::cout<<"Reset"<<std::endl;
			twist->mode = 0;
		}
```
