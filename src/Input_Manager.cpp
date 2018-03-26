#include "WPILib.h"
#include "Input_Manager.hpp"

FRC::Input_Manager::Input_Manager():
	Stick(0),
	Controller(1),
	Switchbox(2),
	Ultrasonic(0),
	Nav(SPI::Port::kMXP),
	PDP(0)

{
	joyXRaw = 0;
	joyYRaw = 0;
	joyZRaw = 0;
	sendspeedX = 0; // The speed that gets sent.
	sendspeedY = 0;
	sendspeedZ = 0;
	goalspeedX = 0; // What speed you're trying to get to.
	goalspeedY = 0;
	goalspeedZ = 0;
	roundsX = 0; // Some delaying stuff, lets me increment the acceleration.
	roundsY = 0;
	roundsZ = 0;
}

double FRC::Input_Manager::xRamp(double joyX)
{
	if (fabs(joyX) > 0.2)
	{
		if (fabs(joyX - goalspeedX) > ACCEL_CAP)
		{
			roundsX = fabs(joyX - goalspeedX) / ACCEL_CAP;
			goalspeedX = joyX;
		}
		else if (fabs(joyX - goalspeedX) > 0.01)
		{
			goalspeedX = joyX;
			sendspeedX = joyX;
			roundsX = 0;
		}

		if (roundsX > 0)
		{
			if((goalspeedX - sendspeedX) > ACCEL_CAP)
			{
				sendspeedX += ACCEL_CAP;
				roundsX--;
			}
			else if((sendspeedX - goalspeedX) > ACCEL_CAP)
			{
				sendspeedX -= ACCEL_CAP;
				roundsX--;
			}
			else
			{
				sendspeedX = goalspeedX;
				roundsX = 0;
			}
			if(sendspeedX >= 1)
			{
				sendspeedX = 1;
				roundsX = 0;
			}
			else if(sendspeedX <= -1)
			{
				sendspeedX = -1;
				roundsX = 0;
			}
		}
		else
		{
			sendspeedX = goalspeedX;
			if(sendspeedX >= 1)//If it's more than one, it is now one.
			{
				sendspeedX = 1;
				goalspeedX = 1;
			}
			else if(sendspeedX <= -1)//If it's less than negative one, it is now negative one.
			{
				sendspeedX = -1;
				sendspeedX = -1;
			}
		}
		return sendspeedX;
	}

	return 0;
}

double FRC::Input_Manager::yRamp(double joyY)//The comments here apply to Xcontroller and Zcontroller, because they are all clones.
{
	if (fabs(joyY) > 0.2)
	{
		if (fabs(joyY - goalspeedY) > ACCEL_CAP)//if you want to change it more than the max acceleration, it won't let you.
		{
			roundsY = fabs(joyY - goalspeedY) / ACCEL_CAP;
			goalspeedY = joyY;//Makes it so it doesn't do the same thing over and over.
		}
		else if (fabs(joyY - goalspeedY) > 0.01)//If you deliberately moved the joystick, it'll change.
		{
			goalspeedY = joyY;
			sendspeedY = joyY;
			roundsY = 0;
		}

		if (roundsY > 0)//Cycles through until you've reached the right speed.
		{
			if((goalspeedY - sendspeedY) > ACCEL_CAP)//If you're less than your goal, it'll accelerate you forward to it.
			{
				sendspeedY += ACCEL_CAP;
				roundsY--;
			}
			else if((sendspeedY - goalspeedY) > ACCEL_CAP)//If you're more than your goal, it'll accelerate you toward it.
			{
				sendspeedY -= ACCEL_CAP;
				roundsY--;
			}
			else//If you're less than an ACCEL_CAP away, it'll just send you to your goal.
			{
				sendspeedY = goalspeedY;
				roundsY = 0;
			}
			if(sendspeedY >= 1)//If it's more than one, it is now one.
			{
				sendspeedY = 1;
				roundsY = 0;
			}
			else if(sendspeedY <= -1)//If it's less than negative one, it is now negative one.
			{
				sendspeedY = -1;
				roundsY = 0;
			}
		}
		else//If you aren't doing your rounds, just set the speed to the goal. Contingency.
		{
			sendspeedY = goalspeedY;
			if(sendspeedY >= 1)//If it's more than one, it is now one.
			{
				sendspeedY = 1;
				goalspeedY = 1;
			}
			else if(sendspeedY <= -1)//If it's less than negative one, it is now negative one.
			{
				sendspeedY = -1;
				sendspeedY = -1;
			}
		}
		return sendspeedY;
	}

	return 0;
}

double FRC::Input_Manager::zRamp(double joyZ)
{
	if(fabs(joyZ) > 0.2)
	{
		if (fabs(joyZ - goalspeedZ) > ACCEL_CAP)
		{
			roundsZ = fabs(joyZ - goalspeedZ) / ACCEL_CAP;
			goalspeedZ = joyZ;
		}
		else if (fabs(joyZ - goalspeedZ) > 0.01)
		{
			goalspeedZ = joyZ;
			sendspeedZ = joyZ;
			roundsZ = 0;
		}

		if (roundsZ > 0)
		{
			if((goalspeedZ - sendspeedZ) > ACCEL_CAP)
			{
				sendspeedZ += ACCEL_CAP;
				roundsZ--;
			}
			else if((sendspeedZ - goalspeedZ) > ACCEL_CAP)
			{
				sendspeedZ -= ACCEL_CAP;
				roundsZ--;
			}
			else
			{
				sendspeedZ = goalspeedZ;
				roundsZ = 0;
			}
			if(sendspeedZ >= 1)
			{
				sendspeedZ = 1;
				roundsZ = 0;
			}
			else if(sendspeedZ <= -1)
			{
				sendspeedZ = -1;
				roundsZ = 0;
			}
		}
		else
		{
			sendspeedZ = goalspeedZ;
			if(sendspeedZ >= 1)//If it's more than one, it is now one.
			{
				sendspeedZ = 1;
				goalspeedZ = 1;
			}
			else if(sendspeedZ <= -1)//If it's less than negative one, it is now negative one.
			{
				sendspeedZ = -1;
				sendspeedZ = -1;
			}
		}
		return sendspeedZ;
	}

	return 0;
}

double FRC::Input_Manager::getAxis(int axis)
{
	return Stick.GetRawAxis(axis);
}

bool FRC::Input_Manager::getJoyButton(int button)
{
	return Stick.GetRawButton(button);
}

double FRC::Input_Manager::getControllerAxis(int axis)
{
	return Controller.GetRawAxis(axis);
}

bool FRC::Input_Manager::getControllerButton(int button)
{
	return Controller.GetRawButton(button);
}

bool FRC::Input_Manager::getSwitch(int button)
{
	return Switchbox.GetRawButton(button);
}

double FRC::Input_Manager::get1220Distance()
{
	return (Ultrasonic.GetVoltage() / 0.0049) / 2.54;
}

double FRC::Input_Manager::getAngle()
{
	return Nav.GetFusedHeading();
}

void FRC::Input_Manager::resetNav()
{
	Nav.Reset();
}

double FRC::Input_Manager::getCurrent(int port)
{
	return PDP.GetCurrent(port);
}
