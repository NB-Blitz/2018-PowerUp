#include "WPILib.h"
#include "Manip_Manager.hpp"

FRC::Manip_Manager::Manip_Manager():
	Manip_Motor(7),
	Arm_In(2),
	Arm_Out(3),
	Left_Intake(5),
	Right_Intake(6)

{

}

void FRC::Manip_Manager::moveManip(double rightControlY)
{
	Manip_Motor.Set(rightControlY);
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

void FRC::Manip_Manager::intake(double leftTrigger, double rightTrigger)
{
	if (leftTrigger > 0.5) {
		Left_Intake.Set(-leftTrigger);
		Right_Intake.Set(leftTrigger);
	}
	else if (rightTrigger > 0.5) {
		Left_Intake.Set(rightTrigger);
		Right_Intake.Set(-rightTrigger);
	}
	else {
		Left_Intake.Set(0);
		Right_Intake.Set(0);
	}
}
