#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"
#include "Auto_Manager.hpp"


class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	FRC::Lift_Manager Lift_Man;
	FRC::AutoManager Auto_Manager;
	double joyX, joyY, joyZ, joySlide, currentAngle;
	double frontSonicDistance, rightSonicDistance, leftSonicDistance;
	AnalogInput frontSonic, rightSonic, leftSonic;
	bool isArcade;
	const double RIGHT_STRAFE_SPEED = -0.5;
	const double LEFT_STRAFE_SPEED = 0.5;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man(),
		Auto_Manager(),
		frontSonic(0),
		rightSonic(1),
		leftSonic(2)
{
		joyX = 0;
		joyY = 0;
		joyZ = 0;
		joySlide = 0;
		currentAngle = 0;
		isArcade = false;
}

	void Autonomous()
	{
		while(IsAutonomous() && IsEnabled())
		{
			joyX = 0;
			joyY = .25;
			joyZ = 0;
			frontSonicDistance = Auto_Manager.convertMB1220SonicVoltageToInches(frontSonic.GetVoltage());
			rightSonicDistance = Auto_Manager.convertMB1013SonicVoltageToInches(rightSonic.GetVoltage());
			leftSonicDistance = Auto_Manager.convertMB1010SonicVoltageToInches(leftSonic.GetVoltage());

			SmartDashboard::PutNumber("FrontSonicVoltage", frontSonic.GetVoltage());
			SmartDashboard::PutNumber("FrontSonicDistance", Auto_Manager.convertMB1220SonicVoltageToInches(frontSonic.GetVoltage()));
			SmartDashboard::PutNumber("RightSonicVoltage", rightSonic.GetVoltage());
			SmartDashboard::PutNumber("RightSonicDistance", Auto_Manager.convertMB1013SonicVoltageToInches(rightSonic.GetVoltage()));
			SmartDashboard::PutNumber("LeftSonicVoltage", leftSonic.GetVoltage());
			SmartDashboard::PutNumber("LeftSonicDistance", Auto_Manager.convertMB1010SonicVoltageToInches(leftSonic.GetVoltage()));

			SmartDashboard::PutNumber("AvoidCollisionOutput",  Auto_Manager.sonicAvoidCollision(frontSonicDistance, rightSonicDistance, leftSonicDistance, joyX, joyY, joyZ));

            if(Auto_Manager.sonicAvoidCollision(frontSonicDistance, rightSonicDistance, leftSonicDistance, joyX, joyY, joyZ) == 0)
			{
				Drive_Man.mecanumDrive(joyX, joyY, joyZ);
			}
			else if(Auto_Manager.sonicAvoidCollision(frontSonicDistance, rightSonicDistance, leftSonicDistance, joyX, joyY, joyZ) == 1)
			{
				Drive_Man.mecanumDrive(RIGHT_STRAFE_SPEED, 0, 0);
			}
			else if(Auto_Manager.sonicAvoidCollision(frontSonicDistance, rightSonicDistance, leftSonicDistance, joyX, joyY, joyZ) == 2)
			{
				Drive_Man.mecanumDrive(LEFT_STRAFE_SPEED, 0, 0);
			}
		}
	}

	/*-----------------------------------------------------------------------------------------------
	 * 	  _______   _              ____          __  __           _
	 *	 |__   __| | |            / __ \        |  \/  |         | |
	 *	    | | ___| | ___ ______| |  | |_ __   | \  / | ___   __| | ___
	 *	    | |/ _ \ |/ _ \______| |  | | '_ \  | |\/| |/ _ \ / _` |/ _ \
	 *	    | |  __/ |  __/      | |__| | |_) | | |  | | (_) | (_| |  __/
	 *	    |_|\___|_|\___|       \____/| .__/  |_|  |_|\___/ \__,_|\___|
	 *	                                | |
	 *	                                |_|
	 *----------------------------------------------------------------------------------------------*/
	void OperatorControl()
	{
		Input_Man.resetNav();

		while (IsOperatorControl() && IsEnabled())
		{
			// VARIABLE SETTING
			//Drive_Man.getEncSpeeds();
			joyX = Input_Man.getAxis(0);
			joyY = -Input_Man.getAxis(1);
			joyZ = Input_Man.getAxis(2);
			joySlide = Input_Man.getAxis(3);
			currentAngle = Input_Man.getAngle();
			isArcade = Input_Man.getJoyButton(1);

			SmartDashboard::PutBoolean("Is Arcade", isArcade);

			// DRIVE
			if (isArcade)
			{
				//Drive_Man.solenoidsOut();
				Drive_Man.arcadeDrive(joyY, joyZ);
			}
			else
			{
				//Drive_Man.solenoidsIn();
				Drive_Man.mecanumDrive(joyX, joyY, joyZ * .7);
			}

			// LIFT
			//Lift_Man.moveLift(joyY);
			//Lift_Man.moveLiftTo(joySlide);

			Wait(0.005);
		}
	}

	/*-----------------------------------------------------------------------------------------------
	 *	  _______        _     __  __           _
	 *	 |__   __|      | |   |  \/  |         | |
	 *	    | | ___  ___| |_  | \  / | ___   __| | ___
	 *	    | |/ _ \/ __| __| | |\/| |/ _ \ / _` |/ _ \
	 *	    | |  __/\__ \ |_  | |  | | (_) | (_| |  __/
	 *	    |_|\___||___/\__| |_|  |_|\___/ \__,_|\___|
	 *
	 *----------------------------------------------------------------------------------------------*/
	void Test()
	{
		while (IsTest() && IsEnabled())
		{
			bool button9 = Input_Man.getJoyButton(9);
			bool button10 = Input_Man.getJoyButton(10);
			bool button11 = Input_Man.getJoyButton(11);
			bool button12 = Input_Man.getJoyButton(12);

			SmartDashboard::PutBoolean("Button 9", button9);
			SmartDashboard::PutBoolean("Button 10", button10);
			SmartDashboard::PutBoolean("Button 11", button11);
			SmartDashboard::PutBoolean("Button 12", button12);

			Drive_Man.testMotorPorts(button9, button10, button11, button12);
		}
	}
};

START_ROBOT_CLASS(Robot)
