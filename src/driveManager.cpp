#include "driveManager.hpp"

FRC::driveManager::driveManager():
	frontRightM(2),
	frontLeftM(0),
	backRightM(3),
	backLeftM(1)

{

}

void FRC::driveManager::tankDrive(double spd, double rotation)
{
	spds[0] = spd + rotation;
	spds[1] = spd - rotation;

	double maxMagnitude = 0;

	for (int i = 0; i < 2; i++)
	{
		double speed = fabs(spds[i]);

		if (maxMagnitude < speed)
		{
			maxMagnitude = speed;
		}
	}

	if (maxMagnitude > 1.0)
	{
		for (int i = 0; i < 2; i++)
		{
			spds[i] = spds[i] / maxMagnitude;
		}
	}

	frontRightM.Set(spds[0]);
	backRightM.Set(spds[0]);

	frontLeftM.Set(spds[1]);
	backLeftM.Set(spds[1]);
}
