#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"
#include "Manip_Manager.hpp"
#include "BlitzLogger.hpp"

class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	//FRC::Lift_Manager Lift_Man;
	//FRC::Manip_Manager Manip_Man;
	FRC::BlitzLogger Blitz_Log;
	double joyX, joyY, joyZ, joySlide, leftControlY, rightControlX, rightControlY, currentAngle;
	bool leftIntake, rightIntake, isArcade;
	bool toggle, mecanumStarted;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Blitz_Log(4)
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
		toggle = false;
		mecanumStarted = false;
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
		double currAngle = Drive_Man.getAngle();
		int timeSinceBegin = 0;
		//Drive_Man.PIDSetupCTRE();
		//Drive_Man.PIDSetup();
		double x = 0;
		Blitz_Log.init();
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
			else if(Input_Man.getJoyButton(9))
			{
				if(!toggle)
				{
					currAngle = Input_Man.getAngle();
					toggle = true;
				}
				Drive_Man.straightDrive(joyX, joyY, currAngle);
			}
			else
			{
				Drive_Man.toMecanum();
				Drive_Man.mecanumDrive(joyX * .5, joyY * .5, joyZ * .3);
				toggle = false;
			}


			// LIFT
//			Lift_Man.moveLift(leftControlY);
//			Lift_Man.moveLiftTo(joySlide);

			// MANIPULATOR
			//Manip_Man.moveManip(rightControlY);
			//Manip_Man.moveArms(rightControlX);
			//Manip_Man.intake(leftIntake, rightIntake);
			//SmartDashboard Variables from Manual PID Correction
			x += 0.002;

			SmartDashboard::PutNumber("PID Out Left Front", Drive_Man.PIDOut[0]);
			SmartDashboard::PutNumber("EncSpeed Left Front", Drive_Man.encSpeed[0]);
			SmartDashboard::PutNumber("P-Component Left Front", Drive_Man.propOut[0]);
			SmartDashboard::PutNumber("I-Component Left Front", Drive_Man.integralOut[0]);
			SmartDashboard::PutNumber("D-Component Left Front", Drive_Man.derivativeOut[0]);
			SmartDashboard::PutNumber("Target Speed Left Front", Drive_Man.targetSpeed[0]);
			SmartDashboard::PutNumber("Current Speed Left Front", Drive_Man.currentSpeed[0]);
			SmartDashboard::PutNumber("Error Left Front", Drive_Man.error[0]);
			SmartDashboard::PutNumber("Goal Speed Left Front", Drive_Man.goalSpeed[0]);
			SmartDashboard::PutNumber("Current Left Front", Drive_Man.Left_Front.GetOutputCurrent());
			SmartDashboard::PutNumber("PID Out Left Back", Drive_Man.PIDOut[1]);
			SmartDashboard::PutNumber("EncSpeed Left Back", Drive_Man.encSpeed[1]);
			SmartDashboard::PutNumber("P-Component Left Back", Drive_Man.propOut[1]);
			SmartDashboard::PutNumber("I-Component Left Back", Drive_Man.integralOut[1]);
			SmartDashboard::PutNumber("D-Component Left Back", Drive_Man.derivativeOut[1]);
			SmartDashboard::PutNumber("Target Speed Left Back", Drive_Man.targetSpeed[1]);
			SmartDashboard::PutNumber("Current Speed Left Back", Drive_Man.currentSpeed[1]);
			SmartDashboard::PutNumber("Error Left Back", Drive_Man.error[1]);
			SmartDashboard::PutNumber("Goal Speed Left Back", Drive_Man.goalSpeed[1]);
			SmartDashboard::PutNumber("Current Left Back", Drive_Man.Left_Back.GetOutputCurrent());
			SmartDashboard::PutNumber("PID Out Right Front", Drive_Man.PIDOut[2]);
			SmartDashboard::PutNumber("EncSpeed Right Front", Drive_Man.encSpeed[2]);
			SmartDashboard::PutNumber("P-Component Right Front", Drive_Man.propOut[2]);
			SmartDashboard::PutNumber("I-Component Right Front", Drive_Man.integralOut[2]);
			SmartDashboard::PutNumber("D-Component Right Front", Drive_Man.derivativeOut[2]);
			SmartDashboard::PutNumber("Target Speed Right Front", Drive_Man.targetSpeed[2]);
			SmartDashboard::PutNumber("Current Speed Right Front", Drive_Man.currentSpeed[2]);
			SmartDashboard::PutNumber("Error Right Front", Drive_Man.error[2]);
			SmartDashboard::PutNumber("Goal Speed Right Front", Drive_Man.goalSpeed[2]);
			SmartDashboard::PutNumber("Current Right Front", Drive_Man.Right_Front.GetOutputCurrent());
			SmartDashboard::PutNumber("PID Out Right Back", Drive_Man.PIDOut[3]);
			SmartDashboard::PutNumber("EncSpeed Right Back", Drive_Man.encSpeed[3]);
			SmartDashboard::PutNumber("P-Component Right Back", Drive_Man.propOut[3]);
			SmartDashboard::PutNumber("I-Component Right Back", Drive_Man.integralOut[3]);
			SmartDashboard::PutNumber("D-Component Right Back", Drive_Man.derivativeOut[3]);
			SmartDashboard::PutNumber("Target Speed Right Back", Drive_Man.targetSpeed[3]);
			SmartDashboard::PutNumber("Current Speed Right Back", Drive_Man.currentSpeed[3]);
			SmartDashboard::PutNumber("Error Right Back", Drive_Man.error[3]);
			SmartDashboard::PutNumber("Goal Speed Right Back", Drive_Man.goalSpeed[3]);
			SmartDashboard::PutNumber("Current Right Back", Drive_Man.Right_Back.GetOutputCurrent());
			SmartDashboard::PutBoolean("PID Disabled", Drive_Man.disablePID);
			/*
			Drive_Man.getEncSpeeds();
			SmartDashboard::PutNumber("EncSpeed Left Front", Drive_Man.encSpeed[0]);
			SmartDashboard::PutNumber("EncSpeed Left Back", Drive_Man.encSpeed[1]);
			SmartDashboard::PutNumber("EncSpeed Right Front", Drive_Man.encSpeed[2]);
			SmartDashboard::PutNumber("EncSpeed Right Back", Drive_Man.encSpeed[3]);
			*/
			//SmartDashboard Variables from Straight Drive
			SmartDashboard::PutNumber("Current Angle", currentAngle);
			//SmartDashboard::PutNumber("Real Angle", Input_Man.getAngle());
			SmartDashboard::PutNumber("Theta", Drive_Man.theta);

			SmartDashboard::PutNumber("Proportional Left Front", Drive_Man.Left_Back.GetClosedLoopError(0));
			SmartDashboard::PutNumber("Integral Left Front", Drive_Man.Left_Back.GetIntegralAccumulator(0));
			SmartDashboard::PutNumber("Derivative Left Front", Drive_Man.Left_Back.GetErrorDerivative(0));

			//SmartDashboard Tester
			/*
			SmartDashboard::PutNumber("X Number Test", x);
			SmartDashboard::PutNumber("Y Number Test", Drive_Man.y);
			SmartDashboard::PutNumber("Prop Test", Drive_Man.propTest);
			SmartDashboard::PutNumber("Integral Test", Drive_Man.intTest);
			SmartDashboard::PutNumber("Deriv Test", Drive_Man.derivTest);
			SmartDashboard::PutNumber("PID Loop Left Back", Drive_Man.PIDLoop(1, joyY, true));
			*/

			timeSinceBegin += 1;
			if ((timeSinceBegin % 600) == 0)
			{
				Blitz_Log.info("Tele-Op", "Left Front Encoder Velocity: " + std::to_string(Drive_Man.encSpeed[0]));
				Blitz_Log.info("Tele-Op", "Left Back Encoder Velocity: " + std::to_string(Drive_Man.encSpeed[1]));
				Blitz_Log.info("Tele-Op", "Right Front Encoder Velocity: " + std::to_string(Drive_Man.encSpeed[2]));
				Blitz_Log.info("Tele-Op", "Right Back Encoder Velocity: " + std::to_string(Drive_Man.encSpeed[3]));
				Blitz_Log.info("Tele-Op", "Left Front Current: " + std::to_string(Drive_Man.Left_Front.GetOutputCurrent()));
				Blitz_Log.info("Tele-Op", "Left Back Current: " + std::to_string(Drive_Man.Left_Back.GetOutputCurrent()));
				Blitz_Log.info("Tele-Op", "Right Front Current: " + std::to_string(Drive_Man.Right_Front.GetOutputCurrent()));
				Blitz_Log.info("Tele-Op", "Right Back Current: " + std::to_string(Drive_Man.Right_Back.GetOutputCurrent()));
				Blitz_Log.info("Tele-Op", "Left Front Given Velocity: " + std::to_string(Drive_Man.finalSpeed[0]));
				Blitz_Log.info("Tele-Op", "Left Back Given Velocity: " + std::to_string(Drive_Man.finalSpeed[1]));
				Blitz_Log.info("Tele-Op", "Right Front Given Velocity: " + std::to_string(Drive_Man.finalSpeed[2]));
				Blitz_Log.info("Tele-Op", "Right Back Given Velocity: " + std::to_string(Drive_Man.finalSpeed[03]));
			}
			 /**/


			Wait(0.005);
		}
		Blitz_Log.close();
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
