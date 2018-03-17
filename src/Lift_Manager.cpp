#include "WPILib.h"
#include "Lift_Manager.hpp"
#include "ctre/Phoenix.h"

FRC::Lift_Manager::Lift_Manager() :
	Lift_Motor(5),
	Encoder(0,1),
	Top_Switch(0),
	Bottom_Switch(1)

{

}

void FRC::Lift_Manager::moveLiftTo(double joyPos)
{
	double height = Encoder.Get();
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

void FRC::Lift_Manager::moveLift(double stickY)
{
	SmartDashboard::PutNumber("Joystick Y", stickY);
	SmartDashboard::PutNumber("Encoder", Encoder.Get());

	if (Top_Switch.Get() && stickY > 0)
	{
		Lift_Motor.Set(0);
	}
	else if (Bottom_Switch.Get() && stickY < 0)
	{
		Lift_Motor.Set(0);
	}
	else
	{
		Lift_Motor.Set(stickY);
	}
}

void FRC::Lift_Manager::resetLift()
{
	SmartDashboard::PutNumber("Encoder", Encoder.Get());

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
	Encoder.Reset();
}
