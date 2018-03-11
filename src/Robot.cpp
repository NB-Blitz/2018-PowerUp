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
		if(!camera_Man.setup)
		{
			camera_Man.camSetup();
		}

		Auto_Man.autoInit(camera_Man);
		Drive_Man.ahrs.Reset();

		SmartDashboard::PutNumber("Starting Pan Pos", camera_Man.pan->Get() * 180);
		SmartDashboard::PutNumber("Starting Tilt Pos ", camera_Man.tilt->Get()* 180);

		camera_Man.camPanPos = camera_Man.pan->Get() * 180;
		camera_Man.camTiltPos = camera_Man.tilt->Get() * 180;

		while(IsAutonomous() && IsEnabled())
		{
			camera_Man.grabData();
			camera_Man.camScan(2);

			SmartDashboard::PutNumber("xPos", camera_Man.xPos);
			SmartDashboard::PutNumber("yPos", camera_Man.yPos);
			SmartDashboard::PutNumber("camTilt", camera_Man.camTiltPos);
			SmartDashboard::PutNumber("actual Tilt", camera_Man.tilt->Get());
			SmartDashboard::PutNumber("camPan", camera_Man.camPanPos - 90);
			SmartDashboard::PutNumber("actual Pan", camera_Man.pan->Get());
			SmartDashboard::PutNumber("front Left", Auto_Man.convertMB1220SonicVoltageToInches(frontLeftSonic.GetVoltage()));
			SmartDashboard::PutNumber("front Right", Auto_Man.convertMB1010SonicVoltageToInches(frontRightSonic.GetVoltage()));
			SmartDashboard::PutNumber("nav angle", Drive_Man.ahrs.GetFusedHeading());


			if(Auto_Man.convertMB1220SonicVoltageToInches(frontLeftSonic.GetVoltage()) > 16 && fabs(camera_Man.xPos) < 8)// > 16 && Auto_Man.convertMB1010SonicVoltageToInches(frontRightSonic.GetVoltage()))
			{
				Auto_Man.driveToCam(.2, camera_Man.angle, camera_Man.targetFound);
			}
//			else if(Auto_Man.convertMB1220SonicVoltageToInches(frontLeftSonic.GetVoltage()) < 16)
//			{
//				Drive_Man.mecanumDrive(0, 0, 0);
//			}
//			else if(Auto_Man.convertMB1010SonicVoltageToInches(frontRightSonic.GetVoltage()) < 16)
//			{
//				Drive_Man.mecanumDrive(0, 0, 0);
//			}
			else
			{

				if(Auto_Man.convertMB1220SonicVoltageToInches(frontLeftSonic.GetVoltage()) < 	16)
				{
					Auto_Man.navStraighten(0);
				}
				else
				{
					Drive_Man.mecanumDrive(0, 0, 0);
				}
			}

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
