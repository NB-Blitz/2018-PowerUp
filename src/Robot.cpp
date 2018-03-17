#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"
#include "Manip_Manager.hpp"
#include "Auto_Manager.hpp"

class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	FRC::Lift_Manager Lift_Man;
	FRC::Manip_Manager Manip_Man;
	FRC::Auto_Manager Auto_Man;
	double joyX, joyY, joyZ, joyDegrees, joySlide;
	double leftControlY, rightControlY, leftTrigger, rightTrigger;
	double currentAngle, ultraDistance, autoStage;
	bool isMecanum, isStraightDrive, isFieldControl;
	bool leftControlButton, rightControlButton;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man(),
		Manip_Man(),
		Auto_Man()

	{
		joyX = 0;
		joyY = 0;
		joyZ = 0;
		joyDegrees = 0;
		joySlide = 0;

		leftControlY = 0;
		rightControlY = 0;
		leftTrigger = 0;
		rightTrigger = 0;

		currentAngle = 0;
		ultraDistance = 0;
		autoStage = 1;

		isMecanum = true;
		isStraightDrive = false;
		isFieldControl = false;

		leftControlButton = false;
		rightControlButton = false;
	}

/*-----------------------------------------------------------------------------------------------
 *                     _
 *		    /\        | |
 *		   /  \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___
 *		  / /\ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __|
 *		 / ____ \ |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \
 *		/_/    \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/
 *
 *----------------------------------------------------------------------------------------------*/
	void Autonomous()
	{
		Input_Man.resetNav();

		while (IsAutonomous() && IsEnabled())
		{
			currentAngle = Input_Man.getAngle();
			ultraDistance = Input_Man.get1220Distance();

			if (Input_Man.getSwitch(1))
			{
				if (autoStage == 1)
				{
					if (Auto_Man.moveForward(5, currentAngle))
					{
						autoStage = 2;
					}
				}
			}

			else if (Input_Man.getSwitch(2))
			{
				if (autoStage == 1)
				{
					if (Auto_Man.moveMiddle(ultraDistance, 18, currentAngle))
					{
						autoStage = 2;
					}
				}
			}

			else if (Input_Man.getSwitch(3))
			{
				if (autoStage == 1)
				{
					if (Auto_Man.moveForward(5, currentAngle))
					{
						autoStage = 2;
					}
				}
			}

			if (Input_Man.getSwitch(4))
			{

			}

			if (Input_Man.getSwitch(5))
			{

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

			// Joystick Values
			joyX = Input_Man.getAxis(0);
			joyY = -Input_Man.getAxis(1);
			joyZ = Input_Man.getAxis(2);
			joyDegrees = Input_Man.Stick.GetDirectionDegrees();

			joySlide = Input_Man.getAxis(3);

			joyX = Input_Man.xRamp(joyX);
			joyY = Input_Man.yRamp(joyY);
			joyZ = Input_Man.zRamp(joyZ);

			isMecanum = !Input_Man.getJoyButton(1);
			isStraightDrive = Input_Man.getJoyButton(2);
			isFieldControl = Input_Man.getJoyButton(3);

			// Controller Values
			leftControlY = Input_Man.getControllerAxis(1);
			rightControlY = Input_Man.getControllerAxis(5);
			leftControlButton = Input_Man.getControllerButton(5);
			rightControlButton = Input_Man.getControllerButton(6);
			leftTrigger = Input_Man.getControllerAxis(2);
			rightTrigger = Input_Man.getControllerAxis(3);

			// Other Sensor Values
			Drive_Man.getEncSpeeds();
			currentAngle = Input_Man.getAngle();

			// SMARTDASHBOARD
			SmartDashboard::PutNumber("Joy X", joyX);
			SmartDashboard::PutNumber("Joy Y", joyX);
			SmartDashboard::PutNumber("Joy Z", joyX);

			SmartDashboard::PutNumber("Left_Front Current", Input_Man.getCurrent(0));
			SmartDashboard::PutNumber("Left_Back Current", Input_Man.getCurrent(2));
			SmartDashboard::PutNumber("Right_Front Current", Input_Man.getCurrent(13));
			SmartDashboard::PutNumber("Right_Back Current", Input_Man.getCurrent(15));

			SmartDashboard::PutNumber("Left_Front Speed", Drive_Man.getEncSpeed(0));
			SmartDashboard::PutNumber("Left_Back Speed", Drive_Man.getEncSpeed(1));
			SmartDashboard::PutNumber("Right_Front Speed", Drive_Man.getEncSpeed(2));
			SmartDashboard::PutNumber("Right_Back Speed", Drive_Man.getEncSpeed(3));

			// DRIVE
			Drive_Man.startCompressor();

			if (isMecanum)
			{
				Drive_Man.switchDriveMode(isMecanum);
				if (isStraightDrive)
				{
					Drive_Man.straightDrive(joyX, joyY, joyZ, currentAngle);
				}
				else if (isFieldControl)
				{
					Drive_Man.fieldControl(joyX, joyY, joyZ, joyDegrees, currentAngle);
				}
				else
				{
					Drive_Man.mecanumDrive(joyX, joyY, joyZ);
				}
			}
			else
			{
				Drive_Man.switchDriveMode(isMecanum);
				Drive_Man.arcadeDrive(joyY, joyZ);
			}

			// LIFT
			Lift_Man.moveLift(leftControlY);

			// MANIPULATOR
			Manip_Man.moveManip(rightControlY);
			Manip_Man.moveArms(leftControlButton, rightControlButton);
			Manip_Man.intake(leftTrigger, rightTrigger);

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
