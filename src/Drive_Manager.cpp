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
	theta = 0;
	rotation = 0;
	startAngle = 0;
	firstTime = true;
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
	finalSpeed[0] = PIDCorrection(baseSpeed[0], encSpeed[0], 0);
	finalSpeed[1] = PIDCorrection(baseSpeed[1], encSpeed[1], 1);
	finalSpeed[2] = PIDCorrection(baseSpeed[2], encSpeed[2], 2);
	finalSpeed[3] = PIDCorrection(baseSpeed[3], encSpeed[3], 3);

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
	finalSpeed[0] = PIDCorrection(baseSpeed[0], encSpeed[0], 0);
	finalSpeed[1] = PIDCorrection(baseSpeed[1], encSpeed[1], 1);
	finalSpeed[2] = PIDCorrection(baseSpeed[2], encSpeed[2], 2);
	finalSpeed[3] = PIDCorrection(baseSpeed[3], encSpeed[3], 3);

	Left_Front.Set(finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}

//Straight Drive Function -> Completely functional (but PID would enhance this)
void FRC::Drive_Manager::straightDrive(double joyX, double joyY, double joyZ, double angle)
{
	if (firstTime)
	{
		firstTime = false;
		startAngle = angle;
	}

	theta = angle;

	if(theta > 180)
	{
		theta = theta - 360;
	}
	else if(theta < -180)
	{
		theta = 360 + theta;
	}

	rotation = (startAngle - theta)/180;

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

double FRC::Drive_Manager::PIDCorrection(double desiredSpeed, double actualSpeed, int motorID) //Manual Version
{
	goalSpeed[motorID] = desiredSpeed * (RATE_FREQUENCY/MAX_HZ); //Speed in terms of encoder
	currentSpeed[motorID] = actualSpeed / RATE_FREQUENCY; //Speed in terms of encoder
	if(motorID > 1)
	{
		currentSpeed[motorID] =  -currentSpeed[motorID];
	}
	error[motorID] = goalSpeed[motorID] - currentSpeed[motorID]; //Error is found
	propOut[motorID] = error[motorID] * PROPORTIONAL_COEFFICIENT[motorID]; //Proportional Gain is proportional to error
	runningIntegral[motorID] += error[motorID] * 0.005; //Integral of error over time is updated
	integralOut[motorID] = INTEGRAL_COEFFICIENT[motorID] * runningIntegral[motorID];
	double derivative = (error[motorID] - preError[motorID]) / 0.005; //Error Derivative is found
	derivativeOut[motorID] = DERIVATIVE_COEFFICIENT[motorID] * derivative;
	PIDOut[motorID] =  propOut[motorID] + integralOut[motorID] + derivativeOut[motorID]; //All components are combined
	targetSpeed[motorID] = goalSpeed[motorID] + PIDOut[motorID];
	preError[motorID] = error[motorID]; //PreError is updated
	return targetSpeed[motorID] / (RATE_FREQUENCY/MAX_HZ); //Returns new speed in -1 to 1 range
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
