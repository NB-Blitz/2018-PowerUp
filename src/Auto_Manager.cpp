#include "Auto_Manager.hpp"
#include "WPILib.h"

FRC::Auto_Manager::Auto_Manager():
	Drive_Man(),
	Input_Man()
{

}

//initializes all the autonomous variables
void FRC::Auto_Manager::autoInit(Camera_Manager Camera_Man)
{
	x += 1;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	SmartDashboard::PutNumber("auto", x);
	SmartDashboard::PutString("gameData", gameData);

	std::string startingPos;
	//std::string startingPos = SmartDashboard::GetString("Autonomous Starting Position", "Center");
	if(Input_Man.getSwitch(1))
	{
		startingPos = "Left";
	}
	else if(Input_Man.getSwitch(2))
	{
		startingPos = "Center";
	}
	else if(Input_Man.getSwitch(3))
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
	if(Input_Man.getSwitch(4))
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
		Camera_Man.setPanPos(90);
	}
	else if(fieldPos == 'L')
	{
		if(autoGoal == 0)
		{
			Camera_Man.setPanPos(Camera_Man.LEFT_SWITCH_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			Camera_Man.setPanPos(Camera_Man.LEFT_SCALE_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SCALE_TILT);
		}
	}
	else if(fieldPos == 'C' && gameData[autoGoal] == 'L')
	{
		if(autoGoal == 0)
		{
			Camera_Man.setPanPos(Camera_Man.CENTER_SWITCH_LEFT_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			Camera_Man.setPanPos(Camera_Man.CENTER_SCALE_LEFT_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SCALE_TILT);
		}
	}
	else if(fieldPos == 'C' && gameData[autoGoal] == 'R')
	{
		if(autoGoal == 0)
		{
			Camera_Man.setPanPos(Camera_Man.CENTER_SWITCH_RIGHT_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			Camera_Man.setPanPos(Camera_Man.CENTER_SCALE_RIGHT_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SCALE_TILT);
		}
	}
	else if(fieldPos == 'R')
	{
		if(autoGoal == 0)
		{
			Camera_Man.setPanPos(Camera_Man.RIGHT_SWITCH_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SWITCH_TILT);
		}
		else if(autoGoal == 1)
		{
			Camera_Man.setPanPos(Camera_Man.RIGHT_SCALE_PAN);
			Camera_Man.setTiltPos(Camera_Man.DEFAULT_SCALE_TILT);
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

//dive to the blob at the center of the camera's view
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

	Drive_Man.mecanumDrive(0, speed, rotation);
	SmartDashboard::PutNumber("Auto Rotation", rotation);
}

//ultrasonic conversions
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

//straightens the robot to and angle
bool FRC::Auto_Manager::navStraighten(double angle)
{
	if(Input_Man.getAngle()-180 > angle + 10)
	{
		Drive_Man.mecanumDrive(0, 0, .15);
		return false;
	}
	else if(Input_Man.getAngle()-180 < angle - 10)
	{
		Drive_Man.mecanumDrive(0, 0, -.15);
		return false;
	}
	else
	{
		Drive_Man.mecanumDrive(0, 0 ,0);
		return true;
	}
}
