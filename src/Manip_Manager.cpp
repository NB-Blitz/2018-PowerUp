#include "WPILib.h"
#include "Manip_Manager.hpp"

FRC::Manip_Manager::Manip_Manager():
	Manip_Motor(7),
	Arm_In(2),
	Arm_Out(3),
	Push(4),
	Withdraw(9),
	Left_Intake(5),
	Right_Intake(6),
	Tilt_Motor(8)

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

void FRC::Manip_Manager::eject(bool on)
{

	if(on)
	{

		moveArms(false, true);
		Withdraw.Set(false);
		Push.Set(true);
	}

	else
	{
		Push.Set(false);
		Withdraw.Set(true);
	}

}

void FRC::Manip_Manager::tiltManip(double xBox)
{
	Tilt_Motor.Set(xBox);
}

void FRC::Manip_Manager::intake(double speed)
{
	Left_Intake.Set(speed);
	Right_Intake.Set(speed);
}

