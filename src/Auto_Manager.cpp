#include "Auto_Manager.hpp"
#include "WPILib.h"

FRC::Auto_Manager::Auto_Manager():
	switch_Box(2),
	drive_Man()
{

}

void FRC::Auto_Manager::autoInit(camera_Manager camera_Man)
{
	x += 1;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	SmartDashboard::PutNumber("auto", x);
	SmartDashboard::PutString("gameData", gameData);

	std::string startingPos;
	//std::string startingPos = SmartDashboard::GetString("Autonomous Starting Position", "Center");
	if(switch_Box.GetRawButton(1))
	{
		startingPos = "Left";
	}
	else if(switch_Box.GetRawButton(2))
	{
		startingPos = "Center";
	}
	else if(switch_Box.GetRawButton(3))
	{
		startingPos = "Right";
	}
	else
	{
		startingPos = "Center";
	}

	SmartDashboard::PutString("starting Position", startingPos);
	std::string autoTarget;

	//autoTarget = SmartDashboard::GetString("Autonomous Destination Selector", "Switch");
	if(switch_Box.GetRawButton(4))
	{
		autoTarget = "Scale";
	}
	else
	{
		autoTarget = "Switch";
	}

	SmartDashboard::PutString("auto Target", autoTarget);


	if(startingPos == "Center")
	{
		fieldPos = 'C';
	}
	else if(startingPos == "Left")
	{
		fieldPos = 'L';
	}
	else if(startingPos == "Right")
	{
		fieldPos = 'R';
	}

	if(autoTarget == "Switch")
	{
		autoGoal = 0;
	}
	else if(autoTarget == "Scale")
	{
		autoGoal = 1;
	}

	SmartDashboard::PutNumber("Auto Goal: ", autoGoal);
	SmartDashboard::PutString("Game Specific Data:", gameData);


	if(fieldPos == gameData[autoGoal])
	{
		camera_Man.setPanPos(90);
	}
	else if(fieldPos == 'L')
	{
		if(autoGoal == 0)
		{
			camera_Man.setPanPos(camera_Man.LEFT_SWITCH_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			camera_Man.setPanPos(camera_Man.LEFT_SCALE_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SCALE_TILT);
		}
	}
	else if(fieldPos == 'C' && gameData[autoGoal] == 'L')
	{
		if(autoGoal == 0)
		{
			camera_Man.setPanPos(camera_Man.CENTER_SWITCH_LEFT_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			camera_Man.setPanPos(camera_Man.CENTER_SCALE_LEFT_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SCALE_TILT);
		}
	}
	else if(fieldPos == 'C' && gameData[autoGoal] == 'R')
	{
		if(autoGoal == 0)
		{
			camera_Man.setPanPos(camera_Man.CENTER_SWITCH_RIGHT_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			camera_Man.setPanPos(camera_Man.CENTER_SCALE_RIGHT_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SCALE_TILT);
		}
	}
	else if(fieldPos == 'R')
	{
		if(autoGoal == 0)
		{
			camera_Man.setPanPos(camera_Man.RIGHT_SWITCH_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			camera_Man.setPanPos(camera_Man.RIGHT_SCALE_PAN);
			camera_Man.setTiltPos(camera_Man.DEFAULT_SCALE_TILT);
		}
	}

	if(autoGoal == 1)
	{
		if(fieldPos == 'R')
		{
			prefferedDogeDir = 1;
		}
		else if(fieldPos == 'L')
		{
			prefferedDogeDir = 2;
		}
		else
		{
			prefferedDogeDir = 0;
		}
	}
	else
	{
		if(gameData[0] == 'R')
		{
			prefferedDogeDir = 1;
		}
		else if(gameData[0] == 'L')
		{
			prefferedDogeDir = 2;
		}
		else
		{
			prefferedDogeDir = 0;
		}
	}

}

void FRC::Auto_Manager::driveToCam(double speed, int angle, bool targetFound)
{
	double rotation = 0;

	if(angle > 10)
	{
		rotation = -.1;
	}
	else if(angle < -10)
	{
		rotation = .1;
	}

	if(!targetFound)
	{
		rotation = 0;
		speed = 0;
	}

	drive_Man.mecanumDrive(0, speed, rotation);
	SmartDashboard::PutNumber("Auto Rotation", rotation);
}

double FRC::Auto_Manager::convertMB1220SonicVoltageToInches(double voltage)
{
	return (((voltage / 0.0049)) / 2.54);
}

double FRC::Auto_Manager::convertMB1013SonicVoltageToInches(double voltage)
{
	return (voltage / (0.00488 / 5) / 25.4);
}

double FRC::Auto_Manager::convertMB1010SonicVoltageToInches(double voltage)
{
	return voltage / 0.0098;
}

void FRC::Auto_Manager::navStraighten(double angle)
{
	if(drive_Man.ahrs.GetFusedHeading()-180 > angle + 10)
	{
		drive_Man.mecanumDrive(0, 0, .15);
	}
	else if(drive_Man.ahrs.GetFusedHeading()-180 < angle - 10)
	{
		drive_Man.mecanumDrive(0, 0, -.15);
	}
	else
	{
		drive_Man.mecanumDrive(0, 0 ,0);
	}
}

