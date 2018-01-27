#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "math.h"

FRC::Drive_Manager::Drive_Manager():
	Input_Man(),

	Left_Back(1),
	Left_Front(0),
	Right_Back(3),
	Right_Front(2),

	Left_Solenoid(0),
	Right_Solenoid(1)

{
	maxMagnitude = 0;
	targetSpeed = 0;
	currentSpeed = 0;
	error = 0;
	propOut = 0;
	PIOut = 0;
	useEnc = false;
}

void FRC::Drive_Manager::arcadeDrive(double joyY, double joyZ)
{
	double speeds[2] = {joyY + joyZ, joyY - joyZ};
	double maxMagnitude = 0;

	for (int i = 0; i < 2; i++)
	{
		double speed = fabs(speeds[i]);

		if (maxMagnitude < speed)
		{
			maxMagnitude = speed;
		}
	}

	if (maxMagnitude > 1)
	{
		for (int i = 0; i < 2; i++)
		{
			speeds[i] /= maxMagnitude;
		}
	}

	Left_Front.Set(speeds[1]);
	Left_Back.Set(speeds[1]);
	Right_Front.Set(speeds[0]);
	Right_Back.Set(speeds[0]);
}

void FRC::Drive_Manager::mecanumDrive(double x, double y, double rotate)
{
	baseSpeed[0] = x + y - rotate;
	baseSpeed[1] = -(-x + y + rotate);
	baseSpeed[2] = -x + y - rotate;
	baseSpeed[3] = -(x + y + rotate);

	for (int i = 1; i < 4; i++)
	{
		double speed = fabs(baseSpeed[i]);
		if (maxMagnitude < speed)
		{
			maxMagnitude = speed;
		}
	}

	if (maxMagnitude > 1.0)
	{
		for (int i = 0; i < 4; i++)
		{
			baseSpeed[i] = baseSpeed[i] / maxMagnitude;
		}
	}

	finalSpeed[0] = PICorrection(baseSpeed[0], encSpeed[0]);
	finalSpeed[1] = PICorrection(baseSpeed[1], encSpeed[1]);
	finalSpeed[2] = PICorrection(baseSpeed[2], encSpeed[2]);
	finalSpeed[3] = PICorrection(baseSpeed[3], encSpeed[3]);

	Left_Front.Set(finalSpeed[0]);
	Left_Back.Set(finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}

double FRC::Drive_Manager::PICorrection(double defaultVal, double encSpeed)
{
	if(useEnc)
	{
		targetSpeed = defaultVal * (RATE_FREQUENCY/MAX_HZ);
		currentSpeed = encSpeed / RATE_FREQUENCY;
		error = targetSpeed - currentSpeed;
		propOut = error * PROPORTIONAL_GAIN;
		PIOut = targetSpeed + propOut;
		return PIOut;
	}
	else
	{
		return defaultVal;
	}
}

void FRC::Drive_Manager::rotate(int degrees)
{
	double turnvalue = Input_Man.getAngle();

	turnvalue += degrees;
	turnvalue %= 360;

	if((degrees > 0) && (leniency(turnvalue)))//Turn right if rotating to the right. Turn left if rotating left.
	{
		Left_Front.Set(rotateSpeed);
		Left_Back.Set(rotateSpeed);
		Right_Front.Set(-rotateSpeed);
		Right_Back.Set(-rotateSpeed);
	}
	else if((degrees < 0) && (leniency(turnvalue)))
	{
		Left_Front.Set(-rotateSpeed);
		Left_Back.Set(-rotateSpeed);
		Right_Front.Set(rotateSpeed);
		Right_Back.Set(rotateSpeed);
	}
	else
	{
		Left_Front.Set(0);
		Left_Back.Set(0);
		Right_Front.Set(0);
		Right_Back.Set(0);
	}
}

bool FRC::Drive_Manager::leniency(int degrees){
	if((Input_Man.getAngle() > (degrees + 1)) || (Input_Man.getAngle() < (degrees - 1)))
	{
		return true;
	}
	else
	{
		return false;
	}
}
void FRC::Drive_Manager::rotateTo(int degrees)
{
	int angle = Input_Man.getAngle();
	int change = degrees - angle;
	rotate(change);
}

void FRC::Drive_Manager::getEncSpeeds()
{
	useEnc = true;
	encSpeed[0] = Left_Front.GetSelectedSensorVelocity(0);
	encSpeed[1] = Left_Back.GetSelectedSensorVelocity(0);
	encSpeed[2] = Right_Front.GetSelectedSensorVelocity(0);
	encSpeed[3] = Right_Back.GetSelectedSensorVelocity(0);
}

double FRC::Drive_Manager::ramp()
{
	if(rampOn)
	{
	rampSpeed = (timer^6) - (3*(timer^4) + (3*(timer^2)));//Had my math guys working on this.
	timer += 0.005;
		if(timer == 1)
		{
			timer = 0;
			rampOn = false;
		}
	return rampSpeed;
	}
}
void FRC::Drive_Manager::rampStart()
{
	rampOn = true;
}
void FRC::Drive_Manager::rampBack()
{
	rampdirection = -1;
}
void FRC::Drive_Manager::rampForward()
{
	rampdirection = 1;
}
void FRC::Drive_Manager::toggleDrive(bool driveType)
{
	Left_Solenoid.Set(driveType);
	Right_Solenoid.Set(driveType);
}
