#include "WPILib.h"
#include "Lift_Manager.hpp"
#include "ctre/Phoenix.h"

FRC::Lift_Manager::Lift_Manager() :
	Lift_Motor(5)

{

}

void FRC::Lift_Manager::moveLiftTo(double joyPos)
{
	double currentHeight = Lift_Motor.GetSelectedSensorPosition(0);
	currentHeight -= (MAX_LIFT_POS/2);
	currentHeight /= (MAX_LIFT_POS/2);

	if(currentHeight < joyPos - .012)
	{
		Lift_Motor.Set(0.5);
	}
	else if(currentHeight > joyPos + .012)
	{
		Lift_Motor.Set(-0.5);
	}
	else
	{
		Lift_Motor.Set(0);
	}
}

void FRC::Lift_Manager::moveLift(double stickY)
{
	Lift_Motor.Set(stickY);
}

void FRC::Lift_Manager::resetLift()
{
	Lift_Motor.Set(-0.2);
	Lift_Motor.SetSelectedSensorPosition(0,0,0);
}
