#include "init.h"

void basic() {
	// picks up the Mobile Goal
	Transport.move_relative(600, 200);
	pros::delay(1000);
	AllAllWheels.move(-55);
	pros::delay(1225);
	MobileGoalManipulator.set_value(true);
	pros::delay(230);
	AllAllWheels.brake();

	// backs up
	PIDMover(3);

	// touches the Ladder
	Transport.move(-128);
	pros::delay(2000);
	PIDTurner(190, 1);

	AllAllWheels.move(40);
	pros::delay(500);
	AllAllWheels.brake();

}