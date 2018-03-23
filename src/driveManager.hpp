#ifndef SRC_DRIVEMANAGER_HPP_
#define SRC_DRIVEMANAGER_HPP_

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <math.h>

namespace FRC
{
	class driveManager
	{
	public:
		driveManager();

		void tankDrive(double spd, double rotation);

		WPI_TalonSRX frontRightM, frontLeftM, backRightM, backLeftM;
		double spds[2];



	private:

	};
}




#endif /* SRC_DRIVEMANAGER_HPP_ */
