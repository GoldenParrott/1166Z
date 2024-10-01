#include "init.h"


void redirect() {
	bool redirectOn = false;
	int redirectStartPoint = 0;

    while (true) {
    // distance sensor (redirect)

		// handles the cases for if R2 is being held down
		if (Master.get_digital(DIGITAL_R2)) {
			// case 1: redirect is currently on
			if (redirectOn == true) {
				// case 1a: if the difference between the starting point and the current point
				// 			is greater than 700 (meaning that it has gone all the way), 
				//			turn off the redirect
				if (abs(Transport.get_position() - redirectStartPoint) >= 1300) {
					redirectOn = false;
					redirectStartPoint = 0;
					Transport.brake();
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					Intake.move(128);
				}
			}
			// case 2: redirect is not on, but the distance sensor is at the proper distance
			else if (Distance.get() < 75) // 
			{
				// in this case, the redirect is started and the starting point is stored for later
				Intake.move(128);
				redirectOn = true;
				redirectStartPoint = Transport.get_position();
			}
			// case 3: if the redirect is not on and should not be on, 
			//		   then L2 moves the robot forward as normal
			else {
				Intake.move(-128);
			}
		// if L2 is not being pressed, then the redirect is turned off
		} else if (!Master.get_digital(DIGITAL_R2)) {
			redirectOn = false;
			redirectStartPoint = 0;
		}
    }
}

void eject() {
	bool ejectOn = false;
	int ejectStartPoint = 0;

	while (true) {
    // distance sensor (eject)

		// handles the cases for if B is being held down
		if (Master.get_digital(DIGITAL_LEFT)) {
			// case 1: redirect is currently on
			if (ejectOn == true) {
				// case 1a: if the difference between the starting point and the current point
				// 			is greater than 700 (meaning that it has gone all the way),
				//			turn off the 
				if (abs(Transport.get_position() - ejectStartPoint) >= 200) {
					ejectOn = false;
					ejectStartPoint = 0;
					Transport.brake();
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					Intake.move(128);
				}
			}
			// case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found the right color
			else if ((((colorSense.get_hue() > 180)                               && (autonnumber > 0)) || // blue
				      ((colorSense.get_hue() < 16) && (colorSense.get_hue() > 10) && (autonnumber < 0)) // red
					 )
					&& (Distance.get() < 75)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(75); // the robot waits for the Ring to reach the proper point before starting the eject
				Intake.move(128);
				ejectOn = true;
				ejectStartPoint = Transport.get_position();
			}
			// case 3: if the redirect is not on and should not be on, 
			//		   then L2 moves the robot forward as normal
			else {
				Intake.move(-128);
			}
		// if L2 is not being pressed, then the redirect is turned off
		} else if (!Master.get_digital(DIGITAL_LEFT)) {
			ejectOn = false;
			ejectStartPoint = 0;
		}
	}
}

void autoEject() {
	bool ejectOn = false;
	int ejectStartPoint = 0;

	while (true) {
    // distance sensor (eject)

		// handles the cases for if B is being held down
			// case 1: redirect is currently on
			if (ejectOn == true) {
				// case 1a: if the difference between the starting point and the current point
				// 			is greater than 700 (meaning that it has gone all the way),
				//			turn off the 
				if (abs(Transport.get_position() - ejectStartPoint) >= 200) {
					ejectOn = false;
					ejectStartPoint = 0;
					Transport.brake();
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					Intake.move(128);
				}
			}
			// case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found the right color
			else if ((((colorSense.get_hue() > 180)                               && (autonnumber > 0)) || // blue
				      ((colorSense.get_hue() < 16) && (colorSense.get_hue() > 10) && (autonnumber < 0)) // red
					 )
					&& (Distance.get() < 75)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(75); // the robot waits for the Ring to reach the proper point before starting the eject
				Intake.move(128);
				ejectOn = true;
				ejectStartPoint = Transport.get_position();
			}
	}
}