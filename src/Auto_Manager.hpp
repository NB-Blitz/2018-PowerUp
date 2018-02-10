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

namespace FRC
{
	class Auto_Manager
	{
	public:
		Auto_Manager();

		FRC::Drive_Manager Drive_Man;
		AHRS ahrs;

		void driveToCam(int angle);



	private:

	};
}



#endif /* SRC_AUTO_MANAGER_HPP_ */
