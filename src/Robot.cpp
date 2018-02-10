#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"
#include "Manip_Manager.hpp"

class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	//FRC::Lift_Manager Lift_Man;
	//FRC::Manip_Manager Manip_Man;
	double joyX, joyY, joyZ, joySlide, leftControlY, rightControlX, rightControlY, currentAngle;
	bool leftIntake, rightIntake, isArcade;

public:
	Robot() :
		Drive_Man(),
		Input_Man()
		//Lift_Man(),
		//Manip_Man()

	{
		joyX = 0;
		joyY = 0;
		joyZ = 0;
		joySlide = 0;
		leftControlY = 0;
		rightControlX = 0;
		rightControlY = 0;
		currentAngle = 0;
		leftIntake = false;
		rightIntake = false;
		isArcade = false;
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
//			Drive_Man.getEncSpeeds();
			joyX = Input_Man.getAxis(0);
			joyY = -Input_Man.getAxis(1);
			joyZ = Input_Man.getAxis(2);
			joySlide = Input_Man.getAxis(3);

			leftControlY = Input_Man.getControllerAxis(1);
			rightControlX = Input_Man.getControllerAxis(4);
			rightControlY = Input_Man.getControllerAxis(5);

			leftIntake = Input_Man.getControllerButton(5);
			rightIntake = Input_Man.getControllerButton(6);

			currentAngle = Input_Man.getAngle();

			isArcade = Input_Man.getJoyButton(1);

//			joyX = Input_Man.prevXRamp(joyX);
//			joyY = Input_Man.prevYRamp(joyY);
//			joyZ = Input_Man.prevZRamp(joyZ);

			joyX = Input_Man.xRamp(joyX);
			joyY = Input_Man.yRamp(joyY);
			joyZ = Input_Man.zRamp(joyZ);

			// DRIVE
			if (isArcade)
			{
//				Drive_Man.toArcade();
				Drive_Man.arcadeDrive(joyY, joyZ);
			}
			else
			{
//				Drive_Man.toMecanum();
				Drive_Man.mecanumDrive(joyX * 0, joyY * .5, joyZ * 0);
			}

			// LIFT
//			Lift_Man.moveLift(leftControlY);
//			Lift_Man.moveLiftTo(joySlide);

			// MANIPULATOR
			//Manip_Man.moveManip(rightControlY);
			//Manip_Man.moveArms(rightControlX);
			//Manip_Man.intake(leftIntake, rightIntake);


			SmartDashboard::PutNumber("Left Front Speed", Drive_Man.Left_Front.Get());
			SmartDashboard::PutNumber("Left Back Speed", Drive_Man.Left_Front.Get());
			SmartDashboard::PutNumber("Right Front Speed", Drive_Man.Left_Front.Get());
			SmartDashboard::PutNumber("Right Back Speed", Drive_Man.Left_Front.Get());
			SmartDashboard::PutNumber("Current_Value", Drive_Man.Left_Front.Get());

			SmartDashboard::PutNumber("Left Front Speed Enc", Drive_Man.Left_Front.GetSelectedSensorVelocity(0) / Drive_Man.RATE_FREQUENCY);
			SmartDashboard::PutNumber("Left Back Speed Enc", Drive_Man.Left_Front.GetSelectedSensorVelocity(0) / Drive_Man.RATE_FREQUENCY);
			SmartDashboard::PutNumber("Right Front Speed Enc", Drive_Man.Left_Front.GetSelectedSensorVelocity(0) / Drive_Man.RATE_FREQUENCY);
			SmartDashboard::PutNumber("Right Back Speed Enc", Drive_Man.Left_Front.GetSelectedSensorVelocity(0) / Drive_Man.RATE_FREQUENCY);
			SmartDashboard::PutNumber("PID_Value", Drive_Man.PIDOut);
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
