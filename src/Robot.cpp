#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"
#include "Manip_Manager.hpp"
#include "Auto_Manager.hpp"
#include "Camera_Manager.hpp"

class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	FRC::Lift_Manager Lift_Man;
	FRC::Manip_Manager Manip_Man;
	FRC::Auto_Manager Auto_Man;
	FRC::Camera_Manager Camera_Man;
	AnalogInput Front_Left_Sonic;
	double joyX, joyY, joyZ, joyDegrees, joySlide;
	double leftControlY, rightControlY, leftTrigger, rightTrigger;
	double currentAngle, ultraDistance, autoStage, liftEncPos;
	bool isMecanum, isStraightDrive, isFieldControl;
	bool leftControlButton, rightControlButton;
	bool inPosition = true;
	bool driveToScale = false;
	bool isStraight = false;
	double timer = 0;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man(),
		Manip_Man(),
		Auto_Man(),
		Camera_Man(),
		Front_Left_Sonic(0)

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
		liftEncPos = 0;

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
		//sets up camera stuff once
		if(!Camera_Man.setup)
		{
			Camera_Man.camSetup();
		}

		//Grabs data from field and Smashboard and determines servo default positioning
		Auto_Man.autoInit(Camera_Man);

		Input_Man.Nav.Reset();

		//Grabs servo positions for the camera manager
		SmartDashboard::PutNumber("Starting Pan Pos", Camera_Man.Pan->Get() * 180);
		SmartDashboard::PutNumber("Starting Tilt Pos ", Camera_Man.Tilt->Get()* 180);
		Camera_Man.camPanPos = Camera_Man.Pan->Get() * 180;
		Camera_Man.camTiltPos = Camera_Man.Tilt->Get() * 180;

		timer = 0;

		while(IsAutonomous() && IsEnabled())
		{
			liftEncPos = Lift_Man.getEncPos();

			if(Input_Man.getSwitch(5))
			{
				//Receives data from pi and moves the camera to the correct position
				Camera_Man.grabData();
				Camera_Man.camScan(2);

				SmartDashboard::PutNumber("run", 3);

				//Gets distance from switch and angle of the bot
				double forwardDist = 0;

				double botAngle = Input_Man.getAngle();

				if(botAngle > 180)
				{
					botAngle = (360 - botAngle) * -1;
				}

				// Drive to the switch and face it
				if(Auto_Man.convertMB1220SonicVoltageToInches(Front_Left_Sonic.GetVoltage()) > 16 && fabs(Camera_Man.xPos) < 8)// > 16 && Auto_Man.convertMB1010SonicVoltageToInches(frontRightSonic.GetVoltage()))
				{
					Auto_Man.driveToCam(.2, Camera_Man.angle, Camera_Man.targetFound);
				}
				else
				{
					if(Auto_Man.convertMB1220SonicVoltageToInches(Front_Left_Sonic.GetVoltage()) < 	16 && Auto_Man.autoGoal == 0 && isStraight)
					{
						isStraight = Auto_Man.navStraighten(0);
					}
					else
					{
						Drive_Man.mecanumDrive(0, 0, 0);

						if(Auto_Man.autoGoal == 0)
						{
							inPosition = true;

						}
						else
						{
							driveToScale = true;
						}
					}
				}

				//starts the path toward the scale
				if(driveToScale)
				{

				}

				//outtakes cube when in position
				if(inPosition)
				{
					if(timer < .5)
					{
						Lift_Man.moveLift(1, liftEncPos);
					}
					else
					{
						Lift_Man.moveLift(0, liftEncPos);
					}

					if(timer > .5 && timer < .7)
					{
						Manip_Man.moveManip(.2);
					}
					else
					{
						Manip_Man.moveManip(0);
					}

					if(timer > .7)
					{
						Manip_Man.intake(1, 0);
					}

					timer += 0.005;
				}

				//SmartDashboard
				SmartDashboard::PutNumber("xPos", Camera_Man.xPos);
				SmartDashboard::PutNumber("yPos", Camera_Man.yPos);
				SmartDashboard::PutNumber("camTilt", Camera_Man.camTiltPos);
				SmartDashboard::PutNumber("actual Tilt", Camera_Man.Tilt->Get());
				SmartDashboard::PutNumber("camPan", Camera_Man.camPanPos - 90);
				SmartDashboard::PutNumber("actual Pan", Camera_Man.Pan->Get());
				SmartDashboard::PutNumber("Switch Dist: ", forwardDist);
				SmartDashboard::PutNumber("nav angle", botAngle);
			}
			else
			{
				timer += .005;

				if(timer < 2)
				{
					Drive_Man.mecanumDrive(0, .4, 0);
				}
				else
				{
					Drive_Man.mecanumDrive(0, 0, 0);
				}
			}

			Wait(0.005);
		}

		//Camera_Man.closeNet();
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
		Lift_Man.resetEnc();

		while (IsOperatorControl() && IsEnabled())
		{
			// VARIABLE SETTING

			// Joystick Values
			joyX = Input_Man.getAxis(0);
			joyY = -Input_Man.getAxis(1);
			joyZ = Input_Man.getAxis(2);
			joyDegrees = Input_Man.Stick.GetDirectionDegrees();

			//joySlide = Input_Man.getAxis(3);

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
			Input_Man.resetNav();
			currentAngle = Input_Man.getAngle();
			liftEncPos = Lift_Man.getEncPos();

			// SMARTDASHBOARD
			SmartDashboard::PutNumber("Joy X", joyX);
			SmartDashboard::PutNumber("Joy Y", joyX);
			SmartDashboard::PutNumber("Joy Z", joyX);

			SmartDashboard::PutNumber("Right Controller Value", rightControlY);

			SmartDashboard::PutNumber("Left_Front Current", Input_Man.getCurrent(0));
			SmartDashboard::PutNumber("Left_Back Current", Input_Man.getCurrent(2));
			SmartDashboard::PutNumber("Right_Front Current", Input_Man.getCurrent(13));
			SmartDashboard::PutNumber("Right_Back Current", Input_Man.getCurrent(15));

			SmartDashboard::PutNumber("Left_Front Speed", Drive_Man.getEncSpeed(0));
			SmartDashboard::PutNumber("Left_Back Speed", Drive_Man.getEncSpeed(1));
			SmartDashboard::PutNumber("Right_Front Speed", Drive_Man.getEncSpeed(2));
			SmartDashboard::PutNumber("Right_Back Speed", Drive_Man.getEncSpeed(3));

			SmartDashboard::PutNumber("Lift Pos", liftEncPos);

			SmartDashboard::PutNumber("Angle", currentAngle);

			// DRIVE
			//Drive_Man.startCompressor();

			if (isMecanum)
			{
				Drive_Man.switchDriveMode(isMecanum);
				if (isStraightDrive)
				{
					Input_Man.resetNav();
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
			Lift_Man.moveLift(leftControlY, liftEncPos);

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
