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

double FRC::Input_Manager::getAxis(int axis)
{
	return stick.GetRawAxis(axis);
}

bool FRC::Input_Manager::getJoyButton(int button)
{
	return stick.GetRawButton(button);
}

void FRC::Input_Manager::resetNav()
{
	nav.Reset();
}
