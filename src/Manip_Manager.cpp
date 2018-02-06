#include "WPILib.h"
#include "Manip_Manager.hpp"

FRC::Manip_Manager::Manip_Manager():
	Manip_Motor(6),
	Arm_Motor(7),
	Left_Spike(0),
	Right_Spike(1)

{

}

void FRC::Manip_Manager::moveManip(double rightControlY)
{
	Manip_Motor.Set(rightControlY * 0.7);
}

void FRC::Manip_Manager::moveArms(double rightControlX)
{
	Arm_Motor.Set(rightControlX * 0.25);
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
