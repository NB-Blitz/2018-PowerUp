#include "Auto_Manager.hpp"

FRC::Auto_Manager::Auto_Manager():
	ahrs {SPI::Port::kMXP},
	Drive_Man()
{

}

void FRC::Auto_Manager::driveToCam(int angle)
{
	Drive_Man.mecanumDrive(0, .15, -angle * 0.01);
}
