#include "WPILib.h"
#include "Manip_Manager.hpp"

FRC::Manip_Manager::Manip_Manager():
	Manip_Motor(7),
	Arm_In(2),
	Arm_Out(3),
	Push(10),
	Withdraw(9),
	Left_Intake(5),
	Right_Intake(6)

{

}

void FRC::Manip_Manager::moveManip(double rightControlY)
{
	Manip_Motor.Set(-rightControlY * 0.8);
}

void FRC::Manip_Manager::moveArms(double leftButton, double rightButton)
{
	if (leftButton > 0.1)
	{
		Arm_In.Set(true);
		Arm_Out.Set(false);
	}
	else if (rightButton > 0.1)
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

void FRC::Manip_Manager::intake()
{

	Left_Intake.Set(1);
	Right_Intake.Set(1);

}
void FRC::Manip_Manager::outtake()
{
	Left_Intake.Set(-1);
	Right_Intake.Set(-1);

}
