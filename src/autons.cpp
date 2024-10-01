#include "init.h"

void blueRingside() {

}


void blueGoalside() {


	PIDMover({-24, -24});

	Transport.move(-65);
	//PIDMover({48, -48}, false);
	//pros::delay(1000);
	//PIDMover({-48, -48}, true);

	pros::delay(500);
}


void redGoalside() {

}


void redRingside() {

	// wut
	pros::Task cancel = pros::Task([]() {PIDTurner(180, 2);});
	cancel.remove();

	// starts the autonomous by raising the arm and moving to the first Ring
	Arm.move_relative(270, 200);
	InputPiston.set_value(true);
	AllWheels.move_relative(130, 100);
	pros::delay(400);

	// picks up the first Ring
	InputMotor.move(-128);
	InputPiston.set_value(false);
	pros::delay(600);
	InputMotor.brake();
	Transport.move_relative(-180, 200);

	// backs up, then turns to the Rings next to the Alliance Stake and moves to them
	PIDMover({53, -10}, true);
	Master.print(0, 0, "ch = %f", getAggregatedHeading(Kalman1, Kalman2));
	double fhol = findHeadingOfLine(universalCurrentLocation, {52, 8});
	pros::delay(100);
	Master.print(1, 0, "gh = %f", fhol);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {52, 8.5}), 2);
	PIDMover({52, 8.5});

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(270, 1);
	AllWheels.move_relative(-360, 100);
	pros::delay(500);
	Transport.move_relative(-280, 200);
	pros::delay(500);

	// moves forward
	PIDMover({48, 0});
	PIDTurner(74, 2);

}

void autonSwitcher(){

	if (autonnumber == -1){
		autonnumber = 1;
	}else if(autonnumber == 1){
		autonnumber = -2;
	}else if(autonnumber == -2){
		autonnumber = 2;
	}else if(autonnumber == 2){
		autonnumber = -1;
	}else{
		autonnumber = 1;
	}

	
/*
	switch (autonnumber) {
	case -2:
		drawRedRing();
		break;
	case -1:
		drawRedMogo();
		break;
	case 1: 
		drawBlueMogo();
		break;
	case 2:
		drawBlueRing();
		break;
	} */
	pros::delay(50);
}