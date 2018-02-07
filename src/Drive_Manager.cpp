#include "WPILib.h"
#include "Drive_Manager.hpp"

FRC::Drive_Manager::Drive_Manager():
	Left_Front(0),
	Left_Back(1),
	Right_Front(2),
	Right_Back(3),

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
	_loops = 0;
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

}

void FRC::Drive_Manager::rotateTo(int degrees)
{

}

void FRC::Drive_Manager::getEncSpeeds()
{
	useEnc = false;
	encSpeed[0] = Left_Front.GetSelectedSensorVelocity(0);
	encSpeed[1] = Left_Back.GetSelectedSensorVelocity(0);
	encSpeed[2] = Right_Front.GetSelectedSensorVelocity(0);
	encSpeed[3] = Right_Back.GetSelectedSensorVelocity(0);
}

void FRC::Drive_Manager::solenoidsOut()
{
	Left_Solenoid.Set(false);
	Right_Solenoid.Set(false);
}

void FRC::Drive_Manager::solenoidsIn()
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

void PIDLoop(double x, double y, double z, bool button)
{

			/* get gamepad axis */
			/*double leftXstick = x;
			double leftYstick = y;
			double leftZstick = z;*/
			/* prepare line to print
			_sb.append("\tout:");
			_sb.append(std::to_string(motorOutput));
			_sb.append("\tspd:");
			_sb.append(std::to_string(_talon->GetSelectedSensorVelocity(kPIDLoopIdx)));
			 while button1 is held down, closed-loop on target velocity */
			if (button)
			{
	        	/* Speed mode */
				/* Convert 500 RPM to units / 100ms.
				 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
				 * velocity setpoint is in units/100ms
				 */
				double Left_Front_targetVelocity_UnitsPer100ms = x+y+z * 500.0 * 4096 / 600;
				double Left_Back_targetVelocity_UnitsPer100ms = -x+y+z * 500.0 * 4096 / 600;
				double Right_Front_targetVelocity_UnitsPer100ms = -x+y-z * 500.0 * 4096 / 600;
				double Right_Back_targetVelocity_UnitsPer100ms = x+y-z * 500.0 * 4096 / 600;
				/* 500 RPM in either direction */

	        	Left_Front.Set(ControlMode::Velocity, Left_Front_targetVelocity_UnitsPer100ms);
	        	Left_Back.Set(ControlMode::Velocity, Left_Back_targetVelocity_UnitsPer100ms);
	        	Right_Front.Set(ControlMode::Velocity, Right_Front_targetVelocity_UnitsPer100ms);
	        	Right_Back.Set(ControlMode::Velocity, Right_Back_targetVelocity_UnitsPer100ms);

				/* append more signals to print when in speed mode.
				_sb.append("\terrNative:");
				_sb.append(std::to_string(_talon->GetClosedLoopError(kPIDLoopIdx)));
				_sb.append("\ttrg:");
				_sb.append(std::to_string(targetVelocity_UnitsPer100ms));*/
	        }
			else
			{
				/* Percent voltage mode */
				Left_Front.Set(ControlMode::PercentOutput, x+y+z);
				Left_Back.Set(ControlMode::PercentOutput, -x+y+z);
				Right_Front.Set(ControlMode::PercentOutput, -x+y-z);
				Right_Back.Set(ControlMode::PercentOutput, x+y-z);
			}
			/* print every ten loops, printing too much too fast is generally bad for performance
			if (++_loops >= 10) {
				_loops = 0;
				printf("%s\n",_sb.c_str());
			}
			_sb.clear();*/

}
