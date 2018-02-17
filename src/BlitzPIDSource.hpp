#ifndef BLITZPIDSOURCE_HPP_
#define BLITZPIDSOURCE_HPP_

#pragma once //File is parsed only once (create objects once)

#include "WPILib.h"
#include "ctre/Phoenix.h"

namespace frc {

	class BlitzPIDSource : public PIDSource // @suppress("Class has a virtual method and non-virtual destructor")
	{
	public:
		WPI_TalonSRX *inputTalon;

		BlitzPIDSource(WPI_TalonSRX & talon);

		double PIDGet();
	};
}





#endif
