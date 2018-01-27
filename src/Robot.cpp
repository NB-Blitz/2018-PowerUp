#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"

class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	FRC::Lift_Manager Lift_Man;
	double joyX, joyY, joyZ, joySlide, currentAngle;
	double oldX, oldY, oldZ, diffX, diffY, diffZ, currentX, currentY, currentZ;
	double currentXSpeed, currentYSpeed, currentZSpeed;
	double unchangedcurrentXSpeed, unchangedcurrentYSpeed, unchangedcurrentZSpeed;
	bool testBoolX, testBoolY, testBoolZ;
	bool toggleDriveButton;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man()

	{
		joyX = 0;
		joyY = 0;
		joyZ = 0;
		oldX = 0;
		oldY = 0;
		oldZ = 0;
		diffX = 0;
		diffY = 0;
		diffZ = 0;
		currentX = 0;
		currentY = 0;
		currentZ = 0;
		currentXSpeed = 0;
		currentYSpeed = 0;
		currentZSpeed = 0;

		unchangedcurrentXSpeed = 0;
		unchangedcurrentYSpeed = 0;
		unchangedcurrentZSpeed = 0;

		testBoolX = true;
		testBoolY = true;
		testBoolZ = true;

		joySlide = 0;
		currentAngle = 0;
		toggleDriveButton = false;
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
			Drive_Man.getEncSpeeds();
			/*joyX = Input_Man.getX();
			joyY = Input_Man.getY();
			joyZ = Input_Man.getZ();*/

			//If change happens, the joystick values will change.
			if(fabs(Input_Man.getX() - currentX) >= 0.05)
			{
				diffX = Input_Man.getX() - currentXSpeed;
				oldX = currentXSpeed;
				currentX = Input_Man.getX();
				Drive_Man.XrampStart();
			}
			else if(fabs(Input_Man.getY() - currentY) >= 0.05)
			{
				diffY = Input_Man.getY() - currentYSpeed;
				oldY = currentYSpeed;
				currentY = Input_Man.getY();
				Drive_Man.YrampStart();
			}
			else if(fabs(Input_Man.getZ() - currentZ) >= 0.05)
			{
				diffZ = Input_Man.getZ() - currentZSpeed;
				oldZ = currentZSpeed;
				currentZ = Input_Man.getZ();
				Drive_Man.ZrampStart();
			}
			joySlide = Input_Man.getSlide();
			currentAngle = Input_Man.getAngle();
			toggleDriveButton = Input_Man.getDriveButton();

//While ramping is occuring, they will ramp. However,
			if(Drive_Man.XrampOn)
			{
				currentXSpeed = oldX + (diffX * Drive_Man.XrampSpeed);
				unchangedcurrentXSpeed = currentXSpeed;
			}
			else
			{
				oldX = unchangedcurrentXSpeed;
			}
			if(Drive_Man.YrampOn)
			{
				currentYSpeed = oldY + (diffY * Drive_Man.YrampSpeed);
				unchangedcurrentYSpeed = currentYSpeed;
			}
			else
			{
				oldY = unchangedcurrentYSpeed;
			}
			if(Drive_Man.ZrampOn)
			{
				currentZSpeed = oldZ + (diffZ * Drive_Man.ZrampSpeed);
				unchangedcurrentZSpeed = currentZSpeed;
			}
			else
			{
				oldZ = unchangedcurrentZSpeed;
			}

			joyX = currentXSpeed;
			joyY = currentYSpeed;
			joyZ = currentZSpeed;
//Encoder+Input testing starts here.
			if((currentYSpeed+currentZSpeed != (Drive_Man.Right_Front) || currentYSpeed+currentZSpeed != (Drive_Man.Right_Back)) && Drive_Man.Left_Solenoid.Get())
			{
				testBoolY = false;
				testBoolZ = false;
			}
			else if((currentYSpeed-currentZSpeed != (Drive_Man.Left_Front) || currentYSpeed+currentZSpeed != (Drive_Man.Left_Back)) && Drive_Man.Right_Solenoid.Get())
						{
				testBoolY = false;
				testBoolZ = false;
			}
			else if(-(currentXSpeed+currentYSpeed+currentZSpeed) != Drive_Man.Right_Back)//3
			{
				testBoolX = false;
				testBoolY = false;
				testBoolZ = false;
			}
			else if(-(-currentXSpeed+currentYSpeed+currentZSpeed) != Drive_Man.Right_Back)//1Left Back
			{
				testBoolX = false;
				testBoolY = false;
				testBoolZ = false;
			}
			else if((-currentXSpeed+currentYSpeed-currentZSpeed) != Drive_Man.Right_Back)//2Right Front
			{
				testBoolX = false;
				testBoolY = false;
				testBoolZ = false;
			}
			else if((currentXSpeed+currentYSpeed-currentZSpeed) != Drive_Man.Right_Back)//0Left Front
			{
				testBoolX = false;
				testBoolY = false;
				testBoolZ = false;
			}
//Put all the testing in the test box? Might make sense to do so.
			Drive_Man.toggleDrive(toggleDriveButton);
			// DRIVE
			if (toggleDriveButton)
			{
				// Pneumatic cylinders out
				Drive_Man.arcadeDrive(joyY, joyZ);
			}
			else
			{
				// Pneumatic cylinders in
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
			Lift_Man.resetLift();
		}
	}
};

START_ROBOT_CLASS(Robot)
