/*
 * Lidar_Manager_V2.hpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Sam
 */

#ifndef SRC_LIDAR_MANAGER_HPP_
#define SRC_LIDAR_MANAGER_HPP_

#include "WPILib.h"
#include "math.h"


namespace FRC
{
	class Lidar_Manager
	{
		public:
			Lidar_Manager();

			//Object Declarations
			FRC::Lidar_Manager *Lidar;
			I2C *Lydar = new I2C(I2C::kOnboard, 0x11);

			//Register Declarations
			uint8_t LIDAR_READ_REGISTER_ADDRESS = 0x10;

			//Multi Register Arrray
			uint8_t LIDAR_START_MOTOR[2] = {0x20, 0x21};
			uint8_t LIDAR_STOP_MOTOR[2] = {0x20, 0x22};

			//Read Buffer Declarations
			uint8_t Lidar_Read_Buffer[4];

			//Function Declarations
			void getLidarValues(uint8_t inputArray[]);

			void startLidarMotor();
			void stopLidarMotor();

	};
}
#endif /* SRC_LIDAR_MANAGER_HPP_ */
