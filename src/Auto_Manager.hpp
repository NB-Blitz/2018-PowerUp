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

		Joystick switch_Box;

		FRC::Drive_Manager drive_Man;

		int autoGoal = 0;
		int x = 0;
		char fieldPos = 'C';

		void autoInit(camera_Manager camera_Man);
		void navStraighten(double angle);
		void driveToCam(double speed, int angle, bool targetFound);
		double convertMB1013SonicVoltageToInches(double voltage);
		double convertMB1010SonicVoltageToInches(double voltage);
		double convertMB1220SonicVoltageToInches(double voltage);

		const int left[2] = {-90, -45};
		const int frontLeft[2] = {-45, 0};
		const int frontRight[2] = {0, 45};
		const int right[2] = {45, 90};

		std::string gameData;

	private:

	};
}



#endif /* SRC_AUTO_MANAGER_HPP_ */
