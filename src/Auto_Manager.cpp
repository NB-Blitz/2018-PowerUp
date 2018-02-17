#include "Auto_Manager.hpp"

FRC::Auto_Manager::Auto_Manager():
	Drive_Man(),
	Camera_Man()
{

}

void FRC::Auto_Manager::driveToCam(int angle)
{
	if(angle != -1)
	{
		Drive_Man.mecanumDrive(0, .25, -angle * 0.004);
	}
	else
	{
		Drive_Man.mecanumDrive(0, 0, 0);
	}
}
