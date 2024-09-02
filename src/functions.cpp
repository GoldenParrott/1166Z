#include "init.h"

/*
void colorSensorOn() {
	int colorDelayTask;
	while (true) {
		//                       < 020
		if (colorSense.get_hue() < 20 && autonnumber > 0) {
			Eject.set_value(true);
			colorDelayTask = 1;
		} else if (colorSense.get_hue() > 210 && autonnumber < 0) {
			Eject.set_value(true);
			colorDelayTask = 1;
		} else if (colorDelayTask >= 500) {
			Eject.set_value(false);
			colorDelayTask = 0;
		}
		if (colorDelayTask != 0) {
			colorDelayTask += 20;
		}
		pros::delay(20);
	}
}
*/

// ensures that the rotational sensor's position is within the range of a heading at all times
void updateOdomPosition() {
	while (true) {
		if (readOdomAngle(RotationalTurn) > 360) {
			RotationalTurn.set_position(0);
		} else if (readOdomAngle(RotationalTurn) < 0) {
			RotationalTurn.set_position(36000);
		}
		pros::delay(5);
	}
}