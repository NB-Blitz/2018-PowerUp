#ifndef SRC_AUTO_MANAGER_HPP_
#define SRC_AUTO_MANAGER_HPP_

#include "WPILib.h"
#include "Drive_Manager.hpp"

namespace FRC
{
	class Auto_Manager
	{
	public:
		Auto_Manager();

		// Objects
		FRC::Drive_Manager Drive_Man;
		Timer Forward_Timer;

		// Methods
		bool moveForward(int seconds, double angle);
		bool moveMiddle(double currentDist, double targetDist, double angle);

		// Variables
	};
}

#endif /* SRC_AUTO_MANAGER_HPP_ */
