#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"
#include "Auto_Manager.hpp"
#include "camera_Manager.hpp"

class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	FRC::Lift_Manager Lift_Man;
	FRC::Auto_Manager Auto_Man;
	FRC::camera_Manager camera_Man;
	AnalogInput frontLeftSonic, frontRightSonic;
	double joyX, joyY, joyZ, joySlide, currentAngle;
	bool isArcade;
	bool inPosition = false;
	bool driveToScale = false;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man(),
		Auto_Man(),
		camera_Man(),
		frontLeftSonic(0),
		frontRightSonic(1)

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

		Drive_Man.ahrs.Reset();

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

			//gets distance from switch and angle of the bot
			double forwardDist;

			double botAngle = Drive_Man.getAngle();

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
			if(Auto_Man.convertMB1220SonicVoltageToInches(frontLeftSonic.GetVoltage()) > 16 && fabs(camera_Man.xPos) < 8)// > 16 && Auto_Man.convertMB1010SonicVoltageToInches(frontRightSonic.GetVoltage()))
			{
				Auto_Man.driveToCam(.2, camera_Man.angle, camera_Man.targetFound);
			}
			else
			{

				if(Auto_Man.convertMB1220SonicVoltageToInches(frontLeftSonic.GetVoltage()) < 	16 && Auto_Man.autoGoal == 0)
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
			SmartDashboard::PutNumber("front Right", Auto_Man.convertMB1010SonicVoltageToInches(frontRightSonic.GetVoltage()));
			SmartDashboard::PutNumber("Switch Dist: ", forwardDist);
			SmartDashboard::PutNumber("nav angle", botAngle);



			Wait(0.005);
		}

		//camera_Man.closeNet();
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

		while (IsOperatorControl() && IsEnabled())
		{
			// VARIABLE SETTING
			//Drive_Man.getEncSpeeds();
			joyX = Input_Man.getAxis(0);
			joyY = -Input_Man.getAxis(1);
			joyZ = Input_Man.getAxis(2);
			joySlide = Input_Man.getAxis(3);
			currentAngle = Drive_Man.getAngle();
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
};
START_ROBOT_CLASS(Robot)
