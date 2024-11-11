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
				/*
				if (abs(Transport.get_position() - redirectStartPoint) >= 1500) {
					redirectOn = false;
					redirectStartPoint = 0;
					Transport.brake();
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else if (abs(Transport.get_position() - redirectStartPoint) >= 1500) {

				} else {
					Intake.move(75);
				}
				*/
			Intake.move(100);
			pros::delay(750);
			redirectOn = false;
			}
			// case 2: redirect is not on, but the distance sensor is at the proper distance
			else if (Distance.get() < 50) {
				// in this case, the redirect is started and the starting point is stored for later
				Intake.move(100);
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
	if(autonnumber < 0){
		Master.print(0,0,"Scoring Red ",NULL);
	}else if(autonnumber > 0){
		Master.print(0,0,"Scoring Blue ",NULL);
	}
	while (true) {
		// distance sensor (eject)
		// Changes the eject to be for the opposite color when the button is pressed
		if (Master.get_digital_new_press(DIGITAL_LEFT)){
			autonnumber *= -1;
			if(autonnumber < 0){
				Master.print(0,0,"Scoring Red ",NULL);
			}else if(autonnumber > 0){
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
			else if ((((colorSense.get_hue() > 180) && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 35)  && (autonnumber > 0)) // red
					 )
					&& (Distance.get() < 50)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(45); // the robot waits for the Ring to reach the proper point before starting the eject
				Transport.move(128);
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
					Transport.move(-128);
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					Transport.move(128);
				}
			}
			// case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found the right color
			else if ((((colorSense.get_hue() > 180) && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 35)  && (autonnumber > 0)) // red
					 )
					&& (Distance.get() < 50)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(65); // the robot waits for the Ring to reach the proper point before starting the eject
				Transport.move(128);
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

void coords() {
	while(1){
		pros::screen::print(TEXT_LARGE, 0, "x = %f",universalCurrentLocation.x);
		pros::screen::print(TEXT_LARGE, 2, "y = %f",universalCurrentLocation.y);
		pros::screen::print(TEXT_LARGE, 4, "θ = %f",getAggregatedHeading(Kalman1, Kalman2));
		pros::delay(100);
	}
}


void CutoffPID(Coordinate goalPoint, bool reverse, double maxAllowableTime) {
	pros::Task movement = pros::Task([goalPoint, reverse] () {PIDMover(goalPoint, reverse);});
	pros::delay(maxAllowableTime);
	movement.remove();
	AllWheels.brake();
}

void CutoffTurnPID(Coordinate goalPoint, bool reverse, double maxAllowableTime, int direction) {
	// the turn movement
	auto movement = [goalPoint, direction] () {PIDTurner(findHeadingOfLine(universalCurrentLocation, goalPoint), direction);};
	// finds the heading's inverse when it is positive
	auto determineInverse = [] (double angle) -> double {
		return angle > 180 ? angle - 180 : angle + 180;
	};
	// the reversed form of the turn movement
	auto revMovement = [goalPoint, direction, determineInverse] () {PIDTurner(determineInverse(findHeadingOfLine(universalCurrentLocation, goalPoint)), direction);};
	pros::Task* movement_task = NULL;
	if (!reverse) {
		movement_task = new pros::Task(movement);
	} else {
		movement_task = new pros::Task(revMovement);
	}
	pros::delay(maxAllowableTime);
	movement_task->remove();
	AllWheels.brake();
}