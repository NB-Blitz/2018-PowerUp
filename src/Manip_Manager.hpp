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
		WPI_TalonSRX Manip_Motor;
		Solenoid Arm_In, Arm_Out, Push, Withdraw;
		Talon Left_Intake, Right_Intake, Tilt_Motor;
		Compressor compressor;

		// Methods
		void moveManip(double rightControlY);
		void moveArms(bool leftButton, bool rightButton);

		void eject(bool on);

		void tiltManip(double xBox);
		void intake(double speed);
	};
}

#endif /* SRC_MANIP_MANAGER_HPP_ */
