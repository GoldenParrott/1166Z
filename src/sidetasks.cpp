#include "init.h"


void redirect() {
	bool redirectOn = false;
	int redirectStartPoint = 0;

    while (true) {
    // distance sensor (redirect)

		int varSpeed = 128;
		int slowdown = 2;
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
			else if (Distance.get() < 200) {
				// in this case, the redirect is started and the starting point is stored for later
				Intake.brake();
				pros::delay(330);
				Intake.move(128);
				redirectOn = true;
				redirectStartPoint = Transport.get_position();
			}
			// case 3: color sensor sees Ring, making it slow down more and more as the robot progresses
			/*
			else if (
					(colorSense.get_hue() > 180) || // blue
					(colorSense.get_hue() < 25) && (colorSense.get_hue() > 10)  // red
					)
			{
				Intake.move(-128 + slowdown);
				slowdown = std::pow(slowdown, 2);
			}
			*/
			// case 4: if the redirect is not on and should not be on, 
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
	int ejectColor = 1;
	Master.print(0,0,"Scoring Blue",NULL);
	while (true) {
		// distance sensor (eject)
		// Changes the eject to be for the opposite color when the button is pressed
		if (Master.get_digital_new_press(DIGITAL_LEFT)){
			if(ejectColor == 1){
				ejectColor = 2;
				Master.print(0,0,"Scoring Red ",NULL);
			}else if(ejectColor == 2){
				ejectColor = 1;
				Master.print(0,0,"Scoring Blue ",NULL);
			}
		}
		// handles the cases for if the eject is in the enabled state
		if(Master.get_digital(DIGITAL_RIGHT)){
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
			//case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found a correct color
			else if ((
					((colorSense.get_hue() > 180)                               && (ejectColor == 2)) || // blue
					((colorSense.get_hue() < 25) && (colorSense.get_hue() > 10) && (ejectColor == 1))   // red
					) && (Distance.get() < 75))
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(60); // the robot waits for the Ring to reach the proper point before starting the eject
				Intake.move(128);
				ejectOn = true;
				ejectStartPoint = Transport.get_position();
			}
			// case 3: if the redirect is not on and should not be on, 
			//		   then L2 moves the robot forward as normal
			else {
				Intake.move(-128);
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
					Intake.move(-128);
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					Intake.move(128);
				}
			}
			// case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found the right color
			else if ((((colorSense.get_hue() > 180) && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 35)  && (autonnumber > 0)) // red
					 )
					&& (Distance.get() < 75)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(65); // the robot waits for the Ring to reach the proper point before starting the eject
				Intake.move(128);
				ejectOn = true;
				ejectStartPoint = Transport.get_position();
			}
	}
}

void unjam() {

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

		}
		else {
			// No problems, continue moving
			Transport.move(128);
		}
	}

	pros::delay(50);
}