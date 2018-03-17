#ifndef SRC_INPUT_MANAGER_HPP_
#define SRC_INPUT_MANAGER_HPP_

#include "WPILib.h"
#include "AHRS.h"

namespace FRC
{
	class Input_Manager
	{
	public:
		Input_Manager();

		// Object Declarations
		Joystick stick;

		// Methods
		double getAngle();
		double getAxis(int axis);
		bool getJoyButton(int button);
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
