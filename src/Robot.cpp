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
	FRC::Auto_Manager Auto_Manager;
	FRC::camera_Manager camera_Man;
	AnalogInput frontSonic;
	double joyX, joyY, joyZ, joySlide, currentAngle;
	bool isArcade;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man(),
		Auto_Manager(),
		camera_Man(),
		frontSonic(0)

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

		camera_Man.pan->Set(.5);
		camera_Man.tilt->Set(.5);

		SmartDashboard::PutNumber("Starting Pan Pos", camera_Man.pan->Get() * 180);
		SmartDashboard::PutNumber("Starting Tilt Pos ", camera_Man.tilt->Get()* 180);

		//Auto_Manager.autoInit();
		camera_Man.camSetup();
		Input_Man.resetNav();

		while(IsAutonomous() && IsEnabled())
		{
			camera_Man.grabData();
			camera_Man.camScan(2);

			SmartDashboard::PutNumber("xPos", camera_Man.xPos);
			SmartDashboard::PutNumber("yPos", camera_Man.yPos);
			SmartDashboard::PutNumber("camTilt", camera_Man.camTiltPos);
			SmartDashboard::PutNumber("actual Tilt", camera_Man.tilt->Get());
			SmartDashboard::PutNumber("camPan", camera_Man.camPanPos);
			SmartDashboard::PutNumber("actual Pan", camera_Man.pan->Get());
			SmartDashboard::PutNumber("Auto Rotation", -(camera_Man.angle-90) * 0.004);
			SmartDashboard::PutNumber("front Dist", Auto_Manager.convertMB1220SonicVoltageToInches(frontSonic.GetVoltage()));


			if(Auto_Manager.convertMB1220SonicVoltageToInches(frontSonic.GetVoltage()) > 24)
			{
				Auto_Manager.driveToCam(.25);
			}
			else
			{
				Drive_Man.mecanumDrive(0, 0, 0);
			}

			Wait(0.005);
		}

		camera_Man.closeNet();
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
};
START_ROBOT_CLASS(Robot)
