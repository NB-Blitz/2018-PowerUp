#include "Auto_Manager.hpp"

FRC::Auto_Manager::Auto_Manager():
	Drive_Man(),
	Camera_Man()
{

}

void FRC::Auto_Manager::driveToCam(int angle)
{
	if(angle != -1)
	{
		Drive_Man.mecanumDrive(0, .25, -angle * 0.004);
	}
	else
	{
		Drive_Man.mecanumDrive(0, 0, 0);
	}
}

double FRC::Auto_Manager::convertMB1220SonicVoltageToInches(double voltage)
{
	return (((voltage / 0.0049)) / 2.54);
}

double FRC::Auto_Manager::convertMB1013SonicVoltageToInches(double voltage)
{
	return (voltage / (0.00488 / 5) / 25.4);
}

double FRC::Auto_Manager::convertMB1010SonicVoltageToInches(double voltage)
{
	return voltage / 0.0098;
}



bool FRC::Auto_Manager::sonicCheckCollision(int distance, double joyStickX, double joyStickY, double joyStickTheta)
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

int FRC::Auto_Manager::sonicAvoidCollision(int frontSonic, int rightSonic, int leftSonic, double joyStickX, double joyStickY, double joyStickTheta)
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


/* Checks to see if the robot will collide with an object with a given direction and distance, then returns which direction to go
 * @parameters
 * double sensorTheta = Direction of sensor reading
 * double sensorDistance = Distance reading from sensor
 * double joyStickX = MecanumDrive x-axis command
 * double joyStickY = mecanumDrive y-axis command
 * double joyStickTheta = Direction of turn from mecanumDrive
 *
 * @returns
 * Boolean stating if current path will result in a collision or not
 */

//bool FRC::Auto_Manager::checkCollision(double sensorTheta, double sensorDistance[], double joyStickX, double joyStickY, double joyStickTheta)
//{
//	if(joyStickY < 0 || joyStickY > 0)
//	{
//		if(sensorTheta <= 45 && sensorTheta >= -45)
//		{
//			if(sensorDistance[sensorTheta] <= MINIMUM_DISTANCE)
//			{
//				return true;
//			}
//		}
//		if(sensorTheta <= -45 && sensorTheta >= -135)
//		{
//			if(sensorDistance[sensorTheta] <= MINIMUM_DISTANCE)
//			{
//				return true;
//			}
//		}
//		if(sensorTheta >= 45 && sensorTheta <= 135)
//		{
//			if(sensorDistance[sensorTheta] <= MINIMUM_DISTANCE)
//			{
//				return true;
//			}
//		}
//	}
//	return false;
//}


//int FRC::Auto_Manager::avoidCollision(double sensorTheta, double sensorDistance[], double joyStickX, double joyStickY, double joyStickTheta)
//{
//	if(!checkCollision(sensorTheta, sensorDistance, joyStickX, joyStickY, joyStickTheta))
//	{
//		return 0;
//	}
//	else if((sensorDistance[sensorTheta + 90] > sensorDistance[sensorTheta - 90]) && !checkCollision(sensorTheta + 90, sensorDistance, joyStickX, joyStickY, joyStickTheta))
//	{
//		return 1;
//	}
//	else if((sensorDistance[sensorTheta + 90] < sensorDistance[sensorTheta - 90]) && !checkCollision(sensorTheta - 90, sensorDistance, joyStickX, joyStickY, joyStickTheta))
//	{
//		return 2;
//	}
//	else
//	{
//		return 3;
//	}
//}

