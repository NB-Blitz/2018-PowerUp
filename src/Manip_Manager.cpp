#include "WPILib.h"
#include "Manip_Manager.hpp"

FRC::Manip_Manager::Manip_Manager():
	Manip_Motor(7),
	Arm_In(2),
	Arm_Out(3)

{

}

void FRC::Manip_Manager::moveManip(double rightControlY)
{
	Manip_Motor.Set(-rightControlY * 0.8);
}

void FRC::Manip_Manager::moveArms(bool leftButton, bool rightButton)
{
	if (leftButton)
	{
		Arm_In.Set(true);
		Arm_Out.Set(false);
	}
	else if (rightButton)
	{
		Arm_In.Set(false);
		Arm_Out.Set(true);
	}
	else
	{
		// DON'T CHANGE
	}
}
