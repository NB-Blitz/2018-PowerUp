#include "WPILib.h"
#include "Auto_Manager.hpp"

FRC::Auto_Manager::Auto_Manager() :
	Drive_Man(),
	Forward_Timer()

{

}

bool FRC::Auto_Manager::moveForward(int seconds, double angle)
{
	Forward_Timer.Start();

	if (Forward_Timer.Get() < seconds)
	{
		Drive_Man.straightDrive(0, 0.5, 0, angle);
	}
	else
	{
		Drive_Man.mecanumDrive(0, 0, 0);
		Forward_Timer.Stop();
		Forward_Timer.Reset();
		return true;
	}

	return false;
}

bool FRC::Auto_Manager::moveMiddle(double currentDist, double targetDist, double angle)
{
	if (currentDist > targetDist)
	{
		Drive_Man.straightDrive(0, 0.5, 0, angle);
	}
	else
	{
		Drive_Man.mecanumDrive(0, 0, 0);
		return true;
	}

	return false;
}
