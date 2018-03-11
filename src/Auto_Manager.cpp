/*
 * Auto_Manager.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Sam
 */


#include "WPILib.h"
#include "Auto_Manager.hpp"

FRC::AutoManager::AutoManager() {}


double FRC::AutoManager::convertMB1220SonicVoltageToInches(double voltage)
{
	return (((voltage / 0.0049)) / 30.48) * 12;
}

double FRC::AutoManager::convertMB1013SonicVoltageToInches(double voltage)
{
	return ((voltage / (0.00488 / 5)) / 25.4);
}

double FRC::AutoManager::convertMB1010SonicVoltageToInches(double voltage)
{
	return ((voltage / 0.0098));
}



bool FRC::AutoManager::sonicCheckCollision(int distance, double joyStickX, double joyStickY, double joyStickTheta)
{

	if(joyStickY > 0)
	{
		if(distance <= MINIMUM_DISTANCE)
		{
			return true;
		}
	}
	return false;
}

int FRC::AutoManager::sonicAvoidCollision(int frontSonic, int rightSonic, int leftSonic, double joyStickX, double joyStickY, double joyStickTheta)
{
	if(!sonicCheckCollision(frontSonic, joyStickX, joyStickY, joyStickTheta))
	{
		return 0;
	}
	else if((rightSonic > leftSonic) && !sonicCheckCollision(rightSonic, joyStickX, joyStickY, joyStickTheta))
	{
		return 1;
	}
	else if((rightSonic < leftSonic) && !sonicCheckCollision(leftSonic, joyStickX, joyStickY, joyStickTheta))
	{
		return 2;
	}
	else
	{
		return 3;
	}
}
