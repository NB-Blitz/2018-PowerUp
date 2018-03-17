/*
 * Lidar_Manager_V2.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Sam
 */
#include <Lidar_Manager.hpp>
#include "WPILib.h"

FRC::Lidar_Manager::Lidar_Manager() : Lidar() {} //Declare Object Ports

//Grabs Distance and Angle Values From RPLidar
void FRC::Lidar_Manager::getLidarValues(uint8_t inputArray[])
{
	Lydar->WriteBulk(&LIDAR_READ_REGISTER_ADDRESS, 1);
	Lydar->ReadOnly(4, inputArray);
}

void FRC::Lidar_Manager::startLidarMotor()
{
	Lydar->Write(0x20, 0x21);

}

void FRC::Lidar_Manager::stopLidarMotor()
{
	Lydar->Write(0x20, 0x22);
}


