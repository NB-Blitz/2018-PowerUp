#ifndef SRC_LIFT_MANAGER_HPP_
#define SRC_LIFT_MANAGER_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"

namespace FRC
{
	class Lift_Manager
	{
	public:
		Lift_Manager();

		// Objects
		WPI_TalonSRX Lift_Motor;
		Encoder Enc;
		DigitalInput Top_Switch, Bottom_Switch;

		// Variables
		int MIN_LIFT_POS = 0;
		int MAX_LIFT_POS = 0;
		int LIFT_RANGE = MAX_LIFT_POS - MIN_LIFT_POS;

		// Methods
		void moveLiftTo(double pos);
		void moveLift(double stickY);
		void resetLift();
		void resetEnc();
	};
}

#endif
