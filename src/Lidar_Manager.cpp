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
uint8_t FRC::Lidar_Manager::getLidarValues()
{
	uint8_t LIDAR_BUFFER = 0;
//	Lydar->Write(1, LIDAR_READ_REGISTER_ADDRESS);
	Lydar->ReadOnly(1, &LIDAR_BUFFER);

	return LIDAR_BUFFER;
}

void FRC::Lidar_Manager::startLidarMotor()
{
	Lydar->Write(0x20, 0x21);
	Lydar->WriteBulk(LIDAR_START_MOTOR, 2);
}

void FRC::Lidar_Manager::stopLidarMotor()
{
	Lydar->Write(0x20, 0x22);
	Lydar->WriteBulk(LIDAR_STOP_MOTOR, 2);
}


