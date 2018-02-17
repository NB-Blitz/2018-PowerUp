#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Math.h"
#include "iostream"
#include "BlitzPIDSource.hpp"

FRC::Drive_Manager::Drive_Manager():
	Left_Front(1),
	Left_Back(2),
	Right_Front(3),
	Right_Back(4),

	Left_Solenoid(0),
	Right_Solenoid(1),
	ahrs {SPI::Port::kMXP}


{


	/* WPILib PID Method Code
	BlitzPIDSource Left_Front_Source(Left_Front);
	BlitzPIDSource Left_Back_Source(Left_Back);
	BlitzPIDSource Right_Front_Source(Right_Front);
	BlitzPIDSource Right_Back_Source(Right_Back);
	PIDController Left_Front_PID(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Left_Front_Source, Left_Front);
	PIDController Left_Back_PID(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Left_Back_Source, Left_Back);
	PIDController Right_Front_PID(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Right_Front_Source, Right_Front);
	PIDController Right_Back_PID(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, Right_Back_Source, Right_Back);
	*/

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
	//Manual PID Loop (Supposed to make the motors run at the same velocity)
	getEncSpeeds();
	numberOfLoops++;
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
		if (fabs(finalSpeed[i]) > 1) {
			finalSpeed[i] = fabs(finalSpeed[i]) / finalSpeed[i]; //Keeps motors within -1 to 1
		}
	}

	Left_Front.Set(-finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}

//Mecanum Drive Function
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
	numberOfLoops++;
	finalSpeed[0] = PIDCorrection(baseSpeed[0], encSpeed[0], 0);
	finalSpeed[1] = PIDCorrection(baseSpeed[1], encSpeed[1], 1);
	finalSpeed[2] = PIDCorrection(baseSpeed[2], encSpeed[2], 2);
	finalSpeed[3] = PIDCorrection(baseSpeed[3], encSpeed[3], 3);
/*
	//Keven's CTRE Method
	finalSpeed[0] = PIDLoop(0, baseSpeed[0], true) + baseSpeed[0];
	finalSpeed[1] = PIDLoop(1, baseSpeed[1], true) + baseSpeed[1];
	finalSpeed[2] = PIDLoop(2, baseSpeed[2], true) + baseSpeed[2];
	finalSpeed[3] = PIDLoop(3, baseSpeed[3], true) + baseSpeed[3];
*/
	/*WPILib PID Method
		Drive_Man.PIDControlWPIlib(finalSpeed[0], finalSpeed[1], finalSpeed[2], finalSpeed[3]);
	*/
	// Deadband
	for (int i = 0; i < 4; i++)
	{
		if (fabs(finalSpeed[i]) < .1)
		{
			finalSpeed[i] = 0;
		}
		if (fabs(finalSpeed[i]) > 1) {
			finalSpeed[i] = fabs(finalSpeed[i]) / finalSpeed[i]; //Keeps motors within -1 to 1
		}
	}


	Left_Front.Set(-finalSpeed[0]);
	Left_Back.Set(-finalSpeed[1]);
	Right_Front.Set(finalSpeed[2]);
	Right_Back.Set(finalSpeed[3]);
}
//Old Motor Correction Method
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
//Manual Motor Correction
double FRC::Drive_Manager::PIDCorrection(double desiredSpeed, double actualSpeed, int motorID) //Manual Version
{
	//if(useEnc)
		//{
			targetSpeed[motorID] = desiredSpeed * (RATE_FREQUENCY/MAX_HZ); //Speed in terms of encoder
			currentSpeed[motorID] = actualSpeed / RATE_FREQUENCY; //Speed in terms of encoder
			error[motorID] = targetSpeed[motorID] - currentSpeed[motorID]; //Error is found
			propOut[motorID] = error[motorID] * -PROPORTIONAL_COEFFICIENT; //Proportional Gain is proportional to error
			runningIntegral[motorID] += error[motorID] * 0.005; //Integral of error over time is updated
			integralOut[motorID] = INTEGRAL_COEFFICIENT * runningIntegral[motorID];
			double derivative = (error[motorID] - preError[motorID]) / 0.005; //Error Derivative is found
			derivativeOut[motorID] = DERIVATIVE_COEFFICIENT * derivative;
			PIDOut[motorID] = targetSpeed[motorID] + propOut[motorID] + integralOut[motorID] + derivativeOut[motorID]; //All components are combined
			preError[motorID] = error[motorID]; //PreError is updated



			if((numberOfLoops % INTEGRAL_RESET_LOOPS) == 0) //Resets Integral Component over time
			{
				runningIntegral[motorID] = 0;
			}
			/* Smart Dashboard Pass/Receive PID Array (Non-functional)
			if (motorID == 0)
			{
				std::vector<double> PID(3);
				std::vector<double> PIDReturn = SmartDashboard::GetNumberArray("PID (Vector)", *PID);
				SmartDashboard::SetDefaultNumberArray("PID (Vector)", PID);

			}
			*/
			return PIDOut[motorID] / (RATE_FREQUENCY/MAX_HZ); //Returns new speed in -1 to 1 range
		//}
		//else
		//{
		//	return desiredSpeed;
		//}
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

//Motor Port Testing Function
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
//Keven's PID Functions
/*
void FRC::Drive_Manager::PIDSetup()//Run it once
{
	Left_Back.Follow(Left_Front);
	Right_Front.Follow(Left_Front);
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
*/
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
//Ryan's CTRE PID Set-up Function
/*
void FRC::Drive_Manager::PIDSetupCTRE()
{
	Left_Front.SelectProfileSlot(0, 0);
	Left_Front.Config_kP(0, PROPORTIONAL_COEFFICIENT, 0.005);
	Left_Front.Config_kI(0, INTEGRAL_COEFFICIENT, 0.005);
	Left_Front.Config_kD(0, DERIVATIVE_COEFFICIENT, 0.005);
	Left_Front.ConfigNominalOutputForward(0, 0.005);
	Left_Front.ConfigNominalOutputReverse(0, 0.005);
	Left_Front.ConfigPeakOutputForward(1, 0.005);
	Left_Front.ConfigPeakOutputReverse(-1, 0.005);

	Left_Back.SelectProfileSlot(0, 0);
	Left_Back.Config_kP(0, PROPORTIONAL_COEFFICIENT, 0.005);
	Left_Back.Config_kI(0, INTEGRAL_COEFFICIENT, 0.005);
	Left_Back.Config_kD(0, DERIVATIVE_COEFFICIENT, 0.005);
	Left_Back.ConfigNominalOutputForward(0, 0.005);
	Left_Back.ConfigNominalOutputReverse(0, 0.005);
	Left_Back.ConfigPeakOutputForward(1, 0.005);
	Left_Back.ConfigPeakOutputReverse(-1, 0.005);

	Right_Front.SelectProfileSlot(0, 0);
	Right_Front.Config_kP(0, PROPORTIONAL_COEFFICIENT, 0.005);
	Right_Front.Config_kI(0, INTEGRAL_COEFFICIENT, 0.005);
	Right_Front.Config_kD(0, DERIVATIVE_COEFFICIENT, 0.005);
	Right_Front.ConfigNominalOutputForward(0, 0.005);
	Right_Front.ConfigNominalOutputReverse(0, 0.005);
	Right_Front.ConfigPeakOutputForward(1, 0.005);
	Right_Front.ConfigPeakOutputReverse(-1, 0.005);

	Right_Back.SelectProfileSlot(0, 0);
	Right_Back.Config_kP(0, PROPORTIONAL_COEFFICIENT, 0.005);
	Right_Back.Config_kI(0, INTEGRAL_COEFFICIENT, 0.005);
	Right_Back.Config_kD(0, DERIVATIVE_COEFFICIENT, 0.005);
	Right_Back.ConfigNominalOutputForward(0, 0.005);
	Right_Back.ConfigNominalOutputReverse(0, 0.005);
	Right_Back.ConfigPeakOutputForward(1, 0.005);
	Right_Back.ConfigPeakOutputReverse(-1, 0.005);
}
*/

/* Current Methods of PID Control:
 * -Old PI Correction Method
 * 	*Not perfect, but is usable
 * -Manual PID Correction
 * 	*Proportional Error is having problems
 * -CTRE PID Correction
 * 	*Untested since corrections
 * -WPILib PID Correction
 * 	*Untested since corrections
 */
