#ifndef SRC_MANIP_MANAGER_HPP_
#define SRC_MANIP_MANAGER_HPP_

#include "WPILIb.h"
#include "ctre/Phoenix.h"

namespace FRC
{
	class Manip_Manager
	{
	public:
		Manip_Manager();

		// Objects
		WPI_TalonSRX Manip_Motor, Left_Intake, Right_Intake;
		Solenoid Arm_In, Arm_Out;

		// Methods
		void moveManip(double rightControlY);
		void intake(bool leftButton, bool rightButton);
	};
}

#endif /* SRC_MANIP_MANAGER_HPP_ */
