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
		Solenoid Arm_In, Arm_Out;
		Talon Left_Intake, Right_Intake;
		Compressor compressor;

		// Methods
		void moveManip(double rightControlY);
		void moveArms(double leftButton, double rightButton);

		void eject(bool on);

		void tiltManip(double xBox);
		void intake();
		void outtake();
	};
}

#endif /* SRC_MANIP_MANAGER_HPP_ */
