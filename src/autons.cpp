#include "init.h"

void basic() {
	// picks up the Mobile Goal
	AllAllWheels.move(40);
	pros::delay(5000);
	AllAllWheels.brake();
	MobileGoalManipulator.set_value(true);

	// touches the Ladder
	AllRightWheels.move(-40);
	AllLeftWheels.move(40);
	pros::delay(2000);
	AllAllWheels.brake();

}