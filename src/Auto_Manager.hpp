/*
 * Auto_Manager.hpp
 *
 *  Created on: Feb 3, 2018
 *      Author: theob
 */

#ifndef SRC_AUTO_MANAGER_HPP_
#define SRC_AUTO_MANAGER_HPP_

#include "Camera_Manager.hpp"
#include "WPILib.h"
#include "AHRS.h"
#include "math.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"

namespace FRC
{
	class Auto_Manager
	{
	public:
		Auto_Manager();

		FRC::Drive_Manager Drive_Man;
		FRC::Input_Manager Input_Man;

		void autoInit(Camera_Manager Camera_Man);
		bool navStraighten(double angle);
		void driveToCam(double speed, int angle, bool targetFound);
		double convertMB1013SonicVoltageToInches(double voltage);
		double convertMB1010SonicVoltageToInches(double voltage);
		double convertMB1220SonicVoltageToInches(double voltage);

		//lidar's seperated sections
		const int left[2] = {-90, -45};
		const int frontLeft[2] = {-45, 0};
		const int frontRight[2] = {0, 45};
		const int right[2] = {45, 90};

		int prefferedDogeDir = 0;
		int autoGoal = 0;
		int x = 0;
		char fieldPos = 'C';

		std::string gameData;
	};
}

#endif /* SRC_AUTO_MANAGER_HPP_ */
