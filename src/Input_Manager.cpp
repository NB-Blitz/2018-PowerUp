#include "WPILib.h"
#include "Input_Manager.hpp"

FRC::Input_Manager::Input_Manager():
	stick(0)

{

}

double FRC::Input_Manager::getAxis(int axis)
{
	return stick.GetRawAxis(axis);
}

bool FRC::Input_Manager::getJoyButton(int button)
{
	return stick.GetRawButton(button);
}
