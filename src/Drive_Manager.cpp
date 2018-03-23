#include "WPILib.h"
#include "Drive_Manager.hpp"

FRC::Drive_Manager::Drive_Manager():
	Left_Front(1),
	Left_Back(2),
	Right_Front(3),
	Right_Back(4),

	Comp(0),

	Left_Solenoid(0),
	Right_Solenoid(1)

{
	maxMagnitude = 0;
	theta = 0;
	rotation = 0;
	targetSpeed = 0;
	currentSpeed = 0;
	error = 0;
	propOut = 0;
	PIOut = 0;
	useEnc = false;
}

void FRC::Drive_Manager::startCompressor()
{
	Comp.SetClosedLoopControl(true);
}

void FRC::Drive_Manager::switchDriveMode(bool isMecanum)
{
	if (isMecanum)
	{
		Left_Solenoid.Set(false);
		Right_Solenoid.Set(true);
	}
	else
	{
		Left_Solenoid.Set(true);
		Right_Solenoid.Set(false);
	}
}

void FRC::Drive_Manager::mecanumDrive(double joyX, double joyY, double joyZ)
{
	// Deadband
	if (fabs(joyX) < 0.2)
	{
		joyX = 0;
	}
	if (fabs(joyY) < 0.2)
	{
		joyY = 0;
	}
	if (fabs(joyZ) < 0.2)
	{
		joyZ = 0;
	}

	baseSpeed[0] = joyX + joyY + joyZ;
	baseSpeed[1] = -joyX + joyY + joyZ;
	baseSpeed[2] = -joyX + joyY - joyZ;
	baseSpeed[3] = joyX + joyY - joyZ;

	maxMagnitude = baseSpeed[0];

	// Sets maxMagnitude to the highest speed out of the 4 motors
	for (int i = 0; i < 4; i++)
	{
		double speed = fabs(baseSpeed[i]);
		if (maxMagnitude < speed)
		{
			maxMagnitude = speed;
		}
	}

	// If maxMagnitude is over 1, divide all 4 motors by maxMagnitude
	// Ensures that the motor speeds are set within -1 and 1
	if (maxMagnitude > 1)
	{
		for (int i = 0; i < 4; i++)
		{
			baseSpeed[i] /= maxMagnitude;
		}
	}

	// PI Loop (Supposed to make the motors run at the same velocity)
	finalSpeed[0] = PICorrection(baseSpeed[0], encSpeed[0]);
	finalSpeed[1] = PICorrection(baseSpeed[1], encSpeed[1]);
	finalSpeed[2] = PICorrection(baseSpeed[2], encSpeed[2]);
	finalSpeed[3] = PICorrection(baseSpeed[3], encSpeed[3]);

	Left_Front.Set(finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}

void FRC::Drive_Manager::arcadeDrive(double joyY, double joyZ)
{
	// Deadband
	if (fabs(joyY) < 0.2)
	{
		joyY = 0;
	}
	if (fabs(joyZ) < 0.2)
	{
		joyZ = 0;
	}

	baseSpeed[0] = joyY + joyZ;
	baseSpeed[1] = joyY + joyZ;
	baseSpeed[2] = joyY - joyZ;
	baseSpeed[3] = joyY - joyZ;

	maxMagnitude = baseSpeed[0];

	// Sets maxMagnitude to the highest speed out of the 4 motors
	for (int i = 0; i < 4; i++)
	{
		double speed = fabs(baseSpeed[i]);

		if (maxMagnitude < speed)
		{
			maxMagnitude = speed;
		}
	}

	// If maxMagnitude is over 1, divide all 4 motors by maxMagnitude
	// Ensures that the motor speeds are set within -1 and 1
	if (maxMagnitude > 1)
	{
		for (int i = 0; i < 4; i++)
		{
			baseSpeed[i] /= maxMagnitude;
		}
	}

	// PI Loop (Supposed to make the motors run at the same velocity)
	finalSpeed[0] = PICorrection(baseSpeed[0], encSpeed[0]);
	finalSpeed[1] = PICorrection(baseSpeed[1], encSpeed[1]);
	finalSpeed[2] = PICorrection(baseSpeed[2], encSpeed[2]);
	finalSpeed[3] = PICorrection(baseSpeed[3], encSpeed[3]);

	Left_Front.Set(finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}

//Straight Drive Function -> Completely functional (but PID would enhance this)
void FRC::Drive_Manager::straightDrive(double joyX, double joyY, double joyZ, double angle)
{
	theta = joyZ - angle;

	if(theta > 180)
	{
		theta = theta - 360;
	}
	else if(theta < -180)
	{
		theta = 360 + theta;
	}

	rotation = theta;

	if(fabs(joyY) > fabs(joyX))
	{
		mecanumDrive(0, joyY, rotation);
	}
	else
	{
		mecanumDrive(joyX, 0, rotation); //Coefficient is lower due to a greater tendency to over-compensate
	}
}

void FRC::Drive_Manager::fieldControl(double joyX, double joyY, double joyZ, double joyDegrees, double angle)
{
	double magnitude = sqrt(pow(joyX, 2) + pow(joyY, 2));

	if(joyDegrees < 0)
	{
		theta = 360 + joyDegrees;
	}
	else
	{
		theta = joyDegrees;
	}

	theta -= angle;

	joyX = magnitude * cos(theta * (PI/180));
	joyY = magnitude * sin(theta * (PI/180));

	mecanumDrive(-joyX, -joyY, joyZ);
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

void FRC::Drive_Manager::getEncSpeeds()
{
	useEnc = false;
	encSpeed[0] = Left_Front.GetSelectedSensorVelocity(0);
	encSpeed[1] = Left_Back.GetSelectedSensorVelocity(0);
	encSpeed[2] = Right_Front.GetSelectedSensorVelocity(0);
	encSpeed[3] = Right_Back.GetSelectedSensorVelocity(0);
}

double FRC::Drive_Manager::getEncSpeed(int motor)
{
	return encSpeed[motor];
}

void FRC::Drive_Manager::testMotorPorts(bool port0, bool port1, bool port2, bool port3)
{
	if (port0)
	{
		Left_Front.Set(1);
	}
	else
	{
		Left_Front.Set(0);
	}

	if (port1)
	{
		Left_Back.Set(1);
	}
	else
	{
		Left_Back.Set(0);
	}

	if (port2)
	{
		Right_Front.Set(1);
	}
	else
	{
		Right_Front.Set(0);
	}

	if (port3)
	{
		Right_Back.Set(1);
	}
	else
	{
		Right_Back.Set(0);
	}
}
