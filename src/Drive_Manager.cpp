#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Math.h"
#include "iostream"
#include "ctre/Phoenix.h"

FRC::Drive_Manager::Drive_Manager():
	Left_Front(1),
	Left_Back(2),
	Right_Front(3),
	Right_Back(4),

	Left_Solenoid(0),
	Right_Solenoid(1),
	ahrs {SPI::Port::kMXP}

{


}

//Get robot's current angle
float FRC::Drive_Manager::getAngle()
{
	return ahrs.GetFusedHeading();
}

//Arcade Drive Function
void FRC::Drive_Manager::arcadeDrive(double joyY, double joyZ)
{
	baseSpeed[0] = joyY + joyZ;
	baseSpeed[1] = joyY + joyZ;
	baseSpeed[2] = joyY - joyZ;
	baseSpeed[3] = joyY - joyZ;

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

	getEncSpeeds(); //Update Enc Speeds

	for (int n = 0; n < 4; n++)
	{
		finalSpeed[n] = PIDCorrection(baseSpeed[n], encSpeed[n], n);
	}

	Left_Front.Set(-finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
	preY = joyY;
	preZ = joyZ;

}

//Mecanum Drive Function
void FRC::Drive_Manager::mecanumDrive(double joyX, double joyY, double joyZ)
{

	baseSpeed[0] = joyX + joyY + joyZ;
	baseSpeed[1] = -joyX + joyY + joyZ;
	baseSpeed[2] = -joyX + joyY - joyZ;
	baseSpeed[3] = joyX + joyY - joyZ;
	y += 0.002;

	// Sets maxMagnitude to the highest speed out of the 4 motors
	maxMagnitude = 0;

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

	//Manual PID Loop (Supposed to make the motors run at the same velocity)
	if ((joyX/preX < 0) || (joyY/preY < 0) || (joyZ/preZ) < 0)
	{
		for(int p = 0; p < 4; p++)
		{
			runningIntegral[p] = 0;
			disablePID = true;
			timeUntilEnablePID = 0.3;
		}
	}
	getEncSpeeds(); //Update Enc Speeds
	numberOfLoops++;

	for (int n = 0; n < 4; n++)
	{
		finalSpeed[n] = PIDCorrection(baseSpeed[n], encSpeed[n], n);
	}


	Left_Front.Set(-finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
	preX = joyX;
	preY = joyY;
	preZ = joyZ;

	angle = getAngle();

	//SmartDashboard::PutNumber("PIDValue", -encSpeed[3]/2000);
	//SmartDashboard::PutNumber("ExpectedValue", baseSpeed[3]);

	double encSpeed2[4];

	encSpeed2[0] = encSpeed[0]/2000;
	encSpeed2[1] = encSpeed[1]/2000;
	encSpeed2[2] = -encSpeed[2]/2000;
	encSpeed2[3] = -encSpeed[3]/2000;

	SmartDashboard::PutNumberArray("Encoder Speed Values", encSpeed2);


	SmartDashboard::PutNumber("LeftFront command", finalSpeed[0]);
	SmartDashboard::PutNumber("LeftBack command", finalSpeed[1]);
	SmartDashboard::PutNumber("RightFront command", finalSpeed[2]);
	SmartDashboard::PutNumber("RightBack command", finalSpeed[3]);

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

//Update Encoder Speeds for PID Correction
void FRC::Drive_Manager::getEncSpeeds()
{
	useEnc = false;
	encSpeed[0] = Left_Front.GetSelectedSensorVelocity(0);
	encSpeed[1] = Left_Back.GetSelectedSensorVelocity(0);
	encSpeed[2] = Right_Front.GetSelectedSensorVelocity(0);
	encSpeed[3] = Right_Back.GetSelectedSensorVelocity(0);
}

//Sets Pneumatics to Arcade Mode
void FRC::Drive_Manager::toArcade()
{
	Left_Solenoid.Set(false);
	Right_Solenoid.Set(false);
}

//Sets Pneumatics to Mecanum Mode
void FRC::Drive_Manager::toMecanum()
{
	Left_Solenoid.Set(true);
	Right_Solenoid.Set(true);
}


//Straight Drive Function -> Completely functional (but PID would enhance this)
void FRC::Drive_Manager::straightDrive(double x, double y, double preAngle)
{
	theta = preAngle - getAngle();

	//theta is pushed within -180 to 180 range
	if(theta > 180)
	{
		theta = theta - 360;
	}
	else if(theta < -180)
	{
		theta = 360 + theta;
	}
		rotation = theta;

	if(fabs(y) > fabs(x))
	{
		mecanumDrive(0, y, rotation * 0.08);
	}
	else
	{
		mecanumDrive(x, 0, rotation * 0.04); //Coefficient is lower due to a greater tendency to over-compensate
	}
}
