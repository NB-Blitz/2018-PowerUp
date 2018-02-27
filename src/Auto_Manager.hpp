/*
 * Auto_Manager.hpp
 *
 *  Created on: Feb 3, 2018
 *      Author: theob
 */

#ifndef SRC_AUTO_MANAGER_HPP_
#define SRC_AUTO_MANAGER_HPP_

#include "WPILib.h"
#include "AHRS.h"
#include "math.h"
#include "Drive_Manager.hpp"
#include "camera_Manager.hpp"

namespace FRC
{
	class Auto_Manager
	{
	public:
		Auto_Manager();

		FRC::Drive_Manager drive_Man;
		FRC::camera_Manager camera_Man;

		int autoGoal = 0;
		char fieldPos = 'C';

		void autoInit();
		void driveToCam(int speed);

		double convertMB1220SonicVoltageToInches(double voltage);

		std::string gameData;

	private:

	};
}



#endif /* SRC_AUTO_MANAGER_HPP_ */
