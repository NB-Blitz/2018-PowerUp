#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Math.h"
#include "iostream"

FRC::Drive_Manager::Drive_Manager():
	Left_Front(1),
	Left_Back(2),
	Right_Front(3),
	Right_Back(4),

	Left_Solenoid(0),
	Right_Solenoid(1),

	Left_Front_Enc(0,1),
	//What are the final two parameter objects? 4th -> gets PID values, 5th -> what is controlled
	Left_Front_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Left_Front_Enc, Left_Front)
	//Left_Back_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Left_Back.SpeedController, Left_Back.SpeedController),
	//Right_Front_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Right_Front.SpeedController, Right_Front.SpeedController),
	//Right_Back_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Right_Back.SpeedController, Right_Back.SpeedController)



{
	maxMagnitude = 0;
	targetSpeed = 0;
	currentSpeed = 0;
	error = 0;
	propOut = 0;
	PIOut = 0;
	integralOut = 0;
	derivativeOut = 0;
	PIDOut = 0;
	useEnc = true;
	Left_Front_Controller.Enable();
}


//WPILib Motor Control Method
/*
void FRC::Drive_Manager::PIDControlWPIlib(double leftFront, double leftBack, double rightFront, double rightBack) {
	Left_Front_Controller.SetSetpoint(leftFront);
	Left_Back_Controller.SetSetpoint(leftBack);
	Right_Front_Controller.SetSetpoint(rightFront);
	Right_Back_Controller.SetSetpoint(rightBack);
}
*/
void FRC::Drive_Manager::arcadeDrive(double joyY, double joyZ)
{
	baseSpeed[0] = joyY + joyZ;
	baseSpeed[1] = joyY + joyZ;
	baseSpeed[2] = joyY - joyZ;
	baseSpeed[3] = joyY - joyZ;

	for (int i = 0; i < 4; i++)
	{
		double speed = fabs(baseSpeed[i]);

		if (maxMagnitude < speed)
		{
			maxMagnitude = speed;
		}
	}

	if (maxMagnitude > 1)
	{
		for (int i = 0; i < 4; i++)
		{
			baseSpeed[i] /= maxMagnitude;
		}
	}

	/* For WPILib Version
	 * for (int j = 0; j < 4; j++)
	 * {
	 * 		if(fabs(baseSpeed[j]) < .1)
	 * 		{
	 * 			baseSpeed[j] = 0;
	 * 		}
	 * }
	 * PIDControlWPILib(baseSpeed[0], baseSpeed[1], baseSpeed[2], baseSpeed[3]);
	 *
	 */


	// PI Loop (Supposed to make the motors run at the same velocity)
	finalSpeed[0] = PICorrection(baseSpeed[0], encSpeed[0]);
	finalSpeed[1] = PICorrection(baseSpeed[1], encSpeed[1]);
	finalSpeed[2] = PICorrection(baseSpeed[2], encSpeed[2]);
	finalSpeed[3] = PICorrection(baseSpeed[3], encSpeed[3]);

	// Deadband
	for (int i = 0; i < 4; i++)
	{
		if (fabs(finalSpeed[i]) < .1)
		{
			finalSpeed[i] = 0;
		}
	}

	Left_Front.Set(-finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}

void FRC::Drive_Manager::mecanumDrive(double joyX, double joyY, double joyZ)
{
	baseSpeed[0] = joyX + joyY + joyZ;
	baseSpeed[1] = -joyX + joyY + joyZ;
	baseSpeed[2] = -joyX + joyY - joyZ;
	baseSpeed[3] = joyX + joyY - joyZ;

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

	/* For WPILib Version
	 * for (int j = 0; j < 4; j++)
	 * {
	 * 		if(fabs(baseSpeed[j]) < .1)
	 * 		{
	 * 			baseSpeed[j] = 0;
	 * 		}
	 * }
	 * PIDControlWPILib(baseSpeed[0], baseSpeed[1], baseSpeed[2], baseSpeed[3]);
	 */

	// PI Loop (Supposed to make the motors run at the same velocity)
	finalSpeed[0] = PIDCorrection(baseSpeed[0], encSpeed[0]);
	finalSpeed[1] = PIDCorrection(baseSpeed[1], encSpeed[1]);
	finalSpeed[2] = PIDCorrection(baseSpeed[2], encSpeed[2]);
	finalSpeed[3] = PIDCorrection(baseSpeed[3], encSpeed[3]);
	std::cout << "Left Front Enc: " << encSpeed[0] << " Left Back Enc: " << encSpeed[1] << "Right Front Enc: " << encSpeed[2] << " Right Back Enc: " << encSpeed[3] <<"\n";


	// Deadband
	for (int i = 0; i < 4; i++)
	{
		if (fabs(finalSpeed[i]) < .1)
		{
			finalSpeed[i] = 0;
		}
	}


	Left_Front.Set(-finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}

double FRC::Drive_Manager::PICorrection(double defaultVal, double encSpeed) //Old Version
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


double FRC::Drive_Manager::PIDCorrection(double desiredSpeed, double actualSpeed) //Manual Version
{
	if(useEnc)
		{
			targetSpeed = desiredSpeed * (RATE_FREQUENCY/MAX_HZ);
			currentSpeed = actualSpeed / RATE_FREQUENCY;
			error = targetSpeed - currentSpeed;
			propOut = error * PROPORTIONAL_COEFFICIENT;
			runningIntegral += error * 0.005;
			integralOut = INTEGRAL_COEFFICIENT * runningIntegral;
			double derivative = (error - preError) / 0.005;
			derivativeOut = DERIVATIVE_COEFFICIENT * derivative;
			PIDOut = targetSpeed + propOut + integralOut + derivativeOut;
			numberOfLoops++;
			preError = error;
			std::cout << "Error: " << error << " Proportional Out: " << propOut << "Integral Out: " << integralOut << "Derivative Out: " << derivativeOut << "\n";
			//double PID[] = {PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT };
			//double PID_Values[] = SmartDashboard::GetNumberArray("PID_Values", *PID); //Look at *PID later
			SmartDashboard::GetNumber("P-Value", PROPORTIONAL_COEFFICIENT);
			SmartDashboard::GetNumber("I-Value", INTEGRAL_COEFFICIENT);
			SmartDashboard::GetNumber("D-Value", DERIVATIVE_COEFFICIENT);
			SmartDashboard::PutNumber("P-Component", propOut);
			SmartDashboard::PutNumber("I-Component", integralOut);
			SmartDashboard::PutNumber("D-Component", derivativeOut);
			if((numberOfLoops % INTEGRAL_RESET_LOOPS) == 0) //Resets Integral Component over time
			{
				runningIntegral = 0;
			}
			return PIDOut;
		}
		else
		{
			return desiredSpeed;
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

void FRC::Drive_Manager::toArcade()
{
	Left_Solenoid.Set(false);
	Right_Solenoid.Set(false);
}

void FRC::Drive_Manager::toMecanum()
{
	Left_Solenoid.Set(true);
	Right_Solenoid.Set(true);
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
