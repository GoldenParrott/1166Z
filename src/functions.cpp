#include "init.h"


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

void colorSensorBlock() {
	while (true) {
		//                       < 020
		if (colorSense.get_hue() < 20) {
			Transport.brake();
		}
	}
}

void raiseArm() {
	UpLeft.tare_position();
	IntakePTO.move(-128);
	waitUntil(abs(UpLeft.get_position()) >= 1586);
	IntakePTO.brake();
}


void lowerArm() {
	UpLeft.tare_position();
	IntakePTO.move(128);
	waitUntil(UpLeft.get_position() <= -1586);
	IntakePTO.brake();
}


void transportThenGripTASK() {
	Transport.move(-128);
	// Odometry again
	// 	      currentMotorReading     gearRatio    wheelCirc  wheelRev singleDeg setPoint
	//                |					  |             |        |        |        |
	waitUntil(BackRight.get_position() * 0.75 * (((3.25 * 3.14) * 2.54) / 360) >= 33);
	GrabPiston.set_value(true);
}



void blockBlueRing() {
	waitUntil(colorSense.get_hue() > 200);
	Transport.brake();
}


void unblockTransport() {
	Transport.brake();
	Transport.tare_position();
	Transport.move_relative(300, 200);
	waitUntil(Transport.get_position() > 300);
	Transport.move(-128);
}