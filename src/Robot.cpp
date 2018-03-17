#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"
#include "Auto_Manager.hpp"
#include "camera_Manager.hpp"
#include "Manip_Manager.hpp"

#include "BlitzLogger.hpp"
#include "Lidar_Manager.hpp"


class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	FRC::Lift_Manager Lift_Man;
	FRC::Auto_Manager Auto_Man;
	FRC::camera_Manager camera_Man;
	FRC::BlitzLogger BlitzLog;
	FRC::Lidar_Manager LidarManager;
	FRC::Manip_Manager Manip_Man;

	double joyX, joyY, joyZ, joySlide, currentAngle;
	bool isArcade;
	bool inPosition = false;
	bool driveToScale = false;
	uint8_t LidarData[4];
	double joyX, joyY, joyZ, joyDegrees, joySlide;
	double leftControlY, rightControlY, leftTrigger, rightTrigger;
	bool isMecanum, isStraightDrive, isFieldControl;
	bool leftControlButton, rightControlButton;

	//Constant Variable Declarations
	const double RIGHT_STRAFE_SPEED = -0.5;
	const double LEFT_STRAFE_SPEED = 0.5;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man(),

		Auto_Man(),
		camera_Man(),
		BlitzLog(4),
		LidarManager()


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
		//sets up camera stuff once
		if(!camera_Man.setup)
		{
			camera_Man.camSetup();
		}

		//grabs data from field and smashboard and determines servo default positioning
		Auto_Man.autoInit(camera_Man);

		Input_Man.resetNav();
		BlitzLog.init();
		LidarManager.startLidarMotor();


		//grabs servo positions for the camera manager
		SmartDashboard::PutNumber("Starting Pan Pos", camera_Man.pan->Get() * 180);
		SmartDashboard::PutNumber("Starting Tilt Pos ", camera_Man.tilt->Get()* 180);
		camera_Man.camPanPos = camera_Man.pan->Get() * 180;
		camera_Man.camTiltPos = camera_Man.tilt->Get() * 180;

		while(IsAutonomous() && IsEnabled())
		{
			//recieves data from pi and moves the camera to the correct position
			camera_Man.grabData();
			camera_Man.camScan(2);

			//receives data from lidar
			LidarManager.getLidarValues(LidarData);

			//gets distance from switch and angle of the bot
			double forwardDist;

			double botAngle = Input_Man.getAngle();

			if(botAngle > 180)
			{
				botAngle = (360 - botAngle) * -1;
			}

			if(botAngle > Auto_Man.left[0] && botAngle < Auto_Man.left[1])
			{
				forwardDist = 1;
			}
			else if(botAngle > Auto_Man.frontLeft[0] && botAngle < Auto_Man.frontLeft[1])
			{
				forwardDist = 2;
			}
			else if(botAngle > Auto_Man.frontRight[0] && botAngle < Auto_Man.frontRight[1])
			{
				forwardDist = 3;
			}
			else if(botAngle + 90 > Auto_Man.right[0] && botAngle < Auto_Man.right[1])
			{
				forwardDist = 4;
			}

			// drive to the switch and position facing it
			if(forwardDist > 16 && fabs(camera_Man.xPos) < 8)// > 16 && Auto_Man.convertMB1010SonicVoltageToInches(frontRightSonic.GetVoltage()))
			{
				Auto_Man.driveToCam(.2, camera_Man.angle, camera_Man.targetFound);
			}
			else
			{

				if(forwardDist < 16 && Auto_Man.autoGoal == 0)
				{
					Auto_Man.navStraighten(0);
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
				//outTake Code Here!!!!!!!!!!!!!
			}

			//smartdashboard stuff
			SmartDashboard::PutNumber("xPos", camera_Man.xPos);
			SmartDashboard::PutNumber("yPos", camera_Man.yPos);
			SmartDashboard::PutNumber("camTilt", camera_Man.camTiltPos);
			SmartDashboard::PutNumber("actual Tilt", camera_Man.tilt->Get());
			SmartDashboard::PutNumber("camPan", camera_Man.camPanPos - 90);
			SmartDashboard::PutNumber("actual Pan", camera_Man.pan->Get());
			SmartDashboard::PutNumber("Switch Dist: ", forwardDist);
			SmartDashboard::PutNumber("nav angle", botAngle);
			SmartDashboard::PutNumber("LidarDataValue1", LidarData[0]);
			SmartDashboard::PutNumber("LIdarDataValue2", LidarData[1]);
			SmartDashboard::PutNumber("LidarDataValue3", LidarData[2]);
			SmartDashboard::PutNumber("LIdarDataValue4", LidarData[3]);



			Wait(0.005);
		}
		BlitzLog.close();
		LidarManager.stopLidarMotor();
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
