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
	ahrs {SPI::Port::kMXP},

	Left_Front_Enc(0,1),
	//What are the final two parameter objects? 4th -> gets PID values, 5th -> what is controlled
	Left_Front_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Left_Front_Enc, Left_Front)
	//Left_Back_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Left_Back.SpeedController, Left_Back.SpeedController),
	//Right_Front_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Right_Front.SpeedController, Right_Front.SpeedController),
	//Right_Back_Controller(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Right_Back.SpeedController, Right_Back.SpeedController)



{

	maxMagnitude = 0;
	PIOut = 0;
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

float FRC::Drive_Manager::getAngle()
{
	return ahrs.GetFusedHeading();
}
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
	getEncSpeeds();
	finalSpeed[0] = PIDCorrection(baseSpeed[0], encSpeed[0], 0);

	finalSpeed[1] = PIDCorrection(baseSpeed[1], encSpeed[1], 1);

	finalSpeed[2] = PIDCorrection(baseSpeed[2], encSpeed[2], 2);

	finalSpeed[3] = PIDCorrection(baseSpeed[3], encSpeed[3], 3);


	// Deadband
	for (int i = 0; i < 4; i++)
	{
		if (fabs(finalSpeed[i]) < .1)
		{
			finalSpeed[i] = 0;
		}
	}

	Left_Front.Set(finalSpeed[0]);
	Left_Back.Set(finalSpeed[1]);
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

	//Manual PID Loop (Supposed to make the motors run at the same velocity)
	getEncSpeeds(); //Update Enc Speeds
	finalSpeed[0] = PIDCorrection(baseSpeed[0], encSpeed[0], 0);
	finalSpeed[1] = PIDCorrection(baseSpeed[1], encSpeed[1], 1);
	finalSpeed[2] = PIDCorrection(baseSpeed[2], encSpeed[2], 2);
	finalSpeed[3] = PIDCorrection(baseSpeed[3], encSpeed[3], 3);
/*
	//CTRE Method
	finalSpeed[0] = PIDLoop(0, baseSpeed[0], true) + baseSpeed[0];
	finalSpeed[1] = PIDLoop(1, baseSpeed[1], true) + baseSpeed[1];
	finalSpeed[2] = PIDLoop(2, baseSpeed[2], true) + baseSpeed[2];
	finalSpeed[3] = PIDLoop(3, baseSpeed[3], true) + baseSpeed[3];
*/
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

/*double FRC::Drive_Manager::PICorrection(double defaultVal, double encSpeed) //Old Version
{
	if(useEnc)
	{
		targetSpeed = defaultVal * (RATE_FREQUENCY/MAX_HZ);
		currentSpeed = encSpeed / RATE_FREQUENCY;
		error = targetSpeed - currentSpeed;
		//propOut = error * PROPORTIONAL_GAIN;
		//PIOut = targetSpeed + propOut;
		return 0;//PIOut;
	}
	else
	{
		return defaultVal;
	}
}
*/

double FRC::Drive_Manager::PIDCorrection(double desiredSpeed, double actualSpeed, int motorID) //Manual Version
{
	//if(useEnc)
		//{
			targetSpeed[motorID] = desiredSpeed * (RATE_FREQUENCY/MAX_HZ);
			currentSpeed[motorID] = actualSpeed / RATE_FREQUENCY;
			error[motorID] = targetSpeed[motorID] - currentSpeed[motorID];
			propOut[motorID] = error[motorID] * -PROPORTIONAL_COEFFICIENT;
			runningIntegral[motorID] += error[motorID] * 0.005;
			integralOut[motorID] = INTEGRAL_COEFFICIENT * runningIntegral[motorID];
			double derivative = (error[motorID] - preError[motorID]) / 0.005;
			derivativeOut[motorID] = DERIVATIVE_COEFFICIENT * derivative;
			PIDOut[motorID] = targetSpeed[motorID] + propOut[motorID] + integralOut[motorID] + derivativeOut[motorID];
			numberOfLoops[motorID]++;
			preError[motorID] = error[motorID];


			SmartDashboard::PutNumber("PID Out Left Front", PIDOut[0]);
			SmartDashboard::PutNumber("EncSpeed Left Front", encSpeed[0]);
			SmartDashboard::PutNumber("P-Component Left Front", propOut[0]);
			SmartDashboard::PutNumber("I-Component Left Front", integralOut[0]);
			SmartDashboard::PutNumber("D-Component Left Front", derivativeOut[0]);
			SmartDashboard::PutNumber("Target Speed Left Front", targetSpeed[0]);


			if((numberOfLoops[motorID] % INTEGRAL_RESET_LOOPS) == 0) //Resets Integral Component over time
			{
				runningIntegral[motorID] = 0;
			}
			if (motorID == 0)
			{
				std::vector<double> PID;
				PID = SmartDashboard::GetNumberArray("PIDValues", *Default_PID); //Look at *PID later
				//SmartDashboard::PutNumber("PIDValue", x);
			}
			return PIDOut[motorID] / (RATE_FREQUENCY/MAX_HZ);
		//}
		//else
		//{
		//	return desiredSpeed;
		//}
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

void FRC::Drive_Manager::PIDSetup()//Run it once
{

	Right_Front.Follow(Left_Front);
	Left_Back.Follow(Left_Front);
	Right_Back.Follow(Left_Front);

}
double FRC::Drive_Manager::PIDLoop(int motorId, double speed, bool straight)
 {
	Left_Front.Set(speed);
	Left_Back.Set(speed);
	Right_Front.Set(speed);
	Right_Back.Set(speed);
	if(straight)
	{

		if(motorId == 1)
		{
			return ((PROPORTIONAL_COEFFICIENT * Left_Front.GetClosedLoopError(1)) + (INTEGRAL_COEFFICIENT * Left_Front.GetIntegralAccumulator(1)) + (DERIVATIVE_COEFFICIENT * Left_Front.GetErrorDerivative(1)));
		}

		else if(motorId == 2)
		{
			return ((PROPORTIONAL_COEFFICIENT * Left_Back.GetClosedLoopError(1)) + (INTEGRAL_COEFFICIENT * Left_Back.GetIntegralAccumulator(1)) + (DERIVATIVE_COEFFICIENT * Left_Back.GetErrorDerivative(1)));
		}
		else if(motorId == 3)
				{
					return ((PROPORTIONAL_COEFFICIENT * Right_Front.GetClosedLoopError(1)) + (INTEGRAL_COEFFICIENT * Right_Front.GetIntegralAccumulator(1)) + (DERIVATIVE_COEFFICIENT * Right_Front.GetErrorDerivative(1)));
				}

				else if(motorId == 4)
				{
					return ((PROPORTIONAL_COEFFICIENT * Right_Back.GetClosedLoopError(1)) + (INTEGRAL_COEFFICIENT * Right_Back.GetIntegralAccumulator(1)) + (DERIVATIVE_COEFFICIENT * Right_Back.GetErrorDerivative(1)));
				}
				else
				{
					return 0;
				}
			}

			else
			{
				if(motorId == 1)
				{
				return ((PROPORTIONAL_COEFFICIENT * Left_Front.GetClosedLoopError(0)) + (INTEGRAL_COEFFICIENT * Left_Front.GetIntegralAccumulator(0)) + (DERIVATIVE_COEFFICIENT * Left_Front.GetErrorDerivative(0)));
			}

				else if(motorId == 2)
				{
					return ((PROPORTIONAL_COEFFICIENT * Left_Back.GetClosedLoopError(0)) + (INTEGRAL_COEFFICIENT * Left_Back.GetIntegralAccumulator(0)) + (DERIVATIVE_COEFFICIENT * Left_Back.GetErrorDerivative(0)));
			}

				else if(motorId == 3)
				{
					return ((PROPORTIONAL_COEFFICIENT * Right_Front.GetClosedLoopError(0)) + (INTEGRAL_COEFFICIENT * Right_Front.GetIntegralAccumulator(0)) + (DERIVATIVE_COEFFICIENT * Right_Front.GetErrorDerivative(0)));
				}

				else if(motorId == 4)
				{
					return ((PROPORTIONAL_COEFFICIENT * Right_Back.GetClosedLoopError(0)) + (INTEGRAL_COEFFICIENT * Right_Back.GetIntegralAccumulator(0)) + (DERIVATIVE_COEFFICIENT * Right_Back.GetErrorDerivative(0)));
				}
				else
				{
					return 0;
				}
			}
		 }

void FRC::Drive_Manager::straightDrive(double x, double y, double preAngle)
{
	delta = preAngle - getAngle();

	if(delta > 180)
	{
		delta = delta - 360;
	}
	else if(delta < -180)
	{
		delta = 360 + delta;
	}
		rotation = delta;

	if(fabs(y) > fabs(x))
	{
		mecanumDrive(0, y, rotation * 0.08);
	}
	else
	{
		mecanumDrive(x, 0, rotation * 0.04);
	}
}


/* Current Methods of PID Control:
 * -Old PI Correction Method
 * 	*Not perfect, but is usable
 * -Manual PID Correction
 * 	*Weird error where all PID contributors are zero, needs work
 * -CTRE PID Correction
 * 	*Untested since corrections
 * -WPILib PID Correction
 *	*Not fully implemented, needs PIDController extended class and other code
 */
