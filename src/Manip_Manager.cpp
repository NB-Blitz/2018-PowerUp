#include "WPILib.h"
#include "Manip_Manager.hpp"

FRC::Manip_Manager::Manip_Manager():
	Manip_Motor(4),
	Left_Spike(0),
	Right_Spike(1)

{

}

void FRC::Manip_Manager::moveArms(double leftButton, double rightButton)
{
	if (leftButton > 1)
	{
		Manip_Motor.Set(leftButton);
	}
	else if (rightButton > 1)
	{
		Manip_Motor.Set(rightButton);
	}
	else
	{
		Manip_Motor.Set(0);
	}
}

void FRC::Manip_Manager::intake(bool leftButton, bool rightButton)
{
	if (leftButton)
	{
		Left_Spike.Set(Relay::kForward);
		Right_Spike.Set(Relay::kForward);
	}
	else if (rightButton)
	{
		Left_Spike.Set(Relay::kReverse);
		Right_Spike.Set(Relay::kReverse);
	}
	else
	{
		Left_Spike.Set(Relay::kOff);
		Right_Spike.Set(Relay::kOff);
	}
}
