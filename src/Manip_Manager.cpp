#include "WPILib.h"
#include "Manip_Manager.hpp"

FRC::Manip_Manager::Manip_Manager():
	Manip_Motor(7),
	Left_Intake(5),
	Right_Intake(6),
	Arm_In(2),
	Arm_Out(3)

{

}

void FRC::Manip_Manager::moveManip(double rightControlY)
{
		Manip_Motor.Set(-rightControlY * 0.8);
}

void FRC::Manip_Manager::intake(bool leftButton, bool rightButton)
{
	if (leftButton)
	{
		Arm_In.Set(true);
		Arm_Out.Set(false);
		Left_Intake.Set(1);
		Right_Intake.Set(1);
	}
	else if (rightButton)
	{
		Arm_In.Set(false);
		Arm_Out.Set(true);
		Left_Intake.Set(-1);
		Right_Intake.Set(-1);
	}
	else if (Arm_In.Get())
	{
		Left_Intake.Set(0.05);
		Right_Intake.Set(0.05);
	}
}
