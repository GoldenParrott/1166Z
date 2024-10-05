#include "init.h"


void redirect() {
	bool redirectOn = false;
	int redirectStartPoint = 0;

    while (true) {
    // distance sensor (redirect)

		int varSpeed = 128;
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
				} else if (abs(Transport.get_position() - redirectStartPoint) >= 1300) {

				} else {
					Intake.move(128);
				}
			}
			// case 2: redirect is not on, but the distance sensor is at the proper distance
			else if (Distance.get() < 100) {
				// in this case, the redirect is started and the starting point is stored for later
				Intake.brake();
				pros::delay(250);
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
	int ejectToggle = -1;
	while (true) {
		// distance sensor (eject)
		// Changes the eject to be in opposite stae whne the button is pressed
		if (Master.get_digital_new_press(DIGITAL_LEFT)){
			ejectToggle = -ejectToggle;
		}
		// handles the cases for if the eject is in the enabled state
		if(Master.get_digital(DIGITAL_RIGHT)){
			if (ejectToggle == 1) {
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
				//case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found the right color
				else if ((((colorSense.get_hue() > 180)                               && (autonnumber < 0)) || // blue
						((colorSense.get_hue() < 25) && (colorSense.get_hue() > 10) && (autonnumber > 0)) // red
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
			// if L2 is in the disabled state, then the redirect is turned off
			} else if (ejectToggle == -1){
				Intake.move(-128);
				ejectOn = false;
				ejectStartPoint = 0;

			}
		pros::delay(20);
		}
	}
}

void autoEject() {
	bool ejectOn = false;
	int ejectStartPoint = 0;

	////////////////////////////////////////////
	//										  //
	//	ERROR: Code is SOMETIMES not reaching //
	//	inside the color sensor statement 	  //
	//	on lines 128-132. Also changed  	  //
	//	red to <25 as it detects better    	  //
	//									      //
	////////////////////////////////////////////

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
			else if ((((colorSense.get_hue() > 180)                               && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 25) && (colorSense.get_hue() > 10) && (autonnumber > 0)) // red
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

void unJam() {

	while(true){
		// Checks if Motor cannot move
		if(Transport.get_torque() > 1){
			pros::delay(500);

			// Secondary check to see if we are stuck
			// or only caught for a moment
			if (Transport.get_torque() > 1){

				// Mpve Transport Backwards
				Transport.move(128);
				pros::delay(250);

			}

		}else{
			// No problems, continue moving
			Transport.move(-128);
		}

		pros::delay(50);
	}
}