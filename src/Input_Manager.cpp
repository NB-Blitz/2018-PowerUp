#include "WPILib.h"
#include "Input_Manager.hpp"

FRC::Input_Manager::Input_Manager():
	nav(SPI::Port::kMXP),
	stick(0)

{

}

double FRC::Input_Manager::getAngle()
{
	return nav.GetFusedHeading();
}

double FRC::Input_Manager::getX()
{
	return stick.GetX();
}

double FRC::Input_Manager::getY()
{
	return stick.GetY();
}

double FRC::Input_Manager::getZ()
{
	return stick.GetRawAxis(2);
}

double FRC::Input_Manager::getSlide()
{
	return stick.GetRawAxis(3);
}

bool FRC::Input_Manager::getDriveButton()
{
	return stick.GetRawButton(0); // 0 ????????? (Trigger)
}

void FRC::Input_Manager::resetNav()
{
	nav.Reset();
}
