#include "WPILib.h"
#include "Lift_Manager.hpp"
#include "ctre/Phoenix.h"

FRC::Lift_Manager::Lift_Manager() :
	Lift_Motor(8),
	Enc(0,1),
	Top_Switch(0),
	Bottom_Switch(1)

{

}

void FRC::Lift_Manager::moveLiftTo(double joyPos)
{
	double height = Enc.Get();
	double convHeight = height - (MAX_LIFT_POS/2);
	convHeight /= (MAX_LIFT_POS/2);

	SmartDashboard::PutNumber("Joystick Pos", joyPos);
	SmartDashboard::PutNumber("Encoder", height);
	SmartDashboard::PutNumber("Encoder (Decimal)", convHeight);

	if(convHeight < joyPos - .012)
	{
		Lift_Motor.Set(.5);
	}
	else if(convHeight > joyPos + .012)
	{
		Lift_Motor.Set(-.5);
	}
	else
	{
		Lift_Motor.Set(0);
	}
}

void FRC::Lift_Manager::moveLift(double stickY, double encPos)
{
	if (fabs(stickY) > 0.1)
	{
		if (fabs(encPos) > 11600 && stickY < 0)
		{
			Lift_Motor.Set(0);
		}
		else
		{
			Lift_Motor.Set(stickY);
		}
	}
	else
	{
		Lift_Motor.Set(0);
	}
}

void FRC::Lift_Manager::resetLift()
{
	SmartDashboard::PutNumber("Encoder", Enc.Get());

	if (!Bottom_Switch.Get())
	{
		Lift_Motor.Set(-0.3);
	}
	else
	{
		Lift_Motor.Set(0);
		resetEnc();
	}
}

void FRC::Lift_Manager::resetEnc()
{
	Lift_Motor.SetSelectedSensorPosition(0, 0, 0);
}

double FRC::Lift_Manager::getEncPos()
{
	return Lift_Motor.GetSelectedSensorPosition(0);
}
