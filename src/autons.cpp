#include "init.h"

void blueRingside() {

	Intake.move(-128);
	AllWheels.move_relative(480,100);
	pros::delay(1000);
	AllWheels.move_relative(-200,100);
	pros::delay(1000);
	AllWheels.move_relative(480,100);
	pros::delay(1000);
	AllWheels.move_relative(-200,100);
	pros::delay(1000);
	AllWheels.move_relative(480,100);
	pros::delay(1000);
	AllWheels.move_relative(-200,100);
	pros::delay(1000);
	AllWheels.move_relative(480,100);
	pros::delay(1000);
	AllWheels.move_relative(-200,100);
	pros::delay(1000);


/*
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};

	// starts the autonomous by raising the arm and moving to the first Ring
	Arm.move_relative(180, 200);
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
	PIDMover({58.5, 17}, true);

	PIDTurner(findHeadingOfLine(universalCurrentLocation, {53.5, 0}), 1);
	PIDMover({53.5, 0});

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(268, 2);
	AllWheels.move_relative(-365, 100);
	pros::delay(500);
	Transport.move_relative(-285, 200);
	pros::delay(500);

	// moves forward from the Alliance Stake
	PIDMover({53.75, 0});
	
	// moves to MoGo
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {70.25, -13.25}), 1);
	PIDMover({28, 21.75}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-180, 200);
	PIDMover({20.5, 27.25}, true, {gripMoGoM}, {5});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {24, 40}), 1);
	Intake.move(-128);
	PIDMover({24, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {11.75, 8.5}), 1);
	InputMotor.brake();
	PIDMover({11.75, 8.5});
*/
}


void blueGoalside() {

	double motorStartPoint = 0;

	auto triggerGrabber = []() {Grabber.set_value(true);};
	auto untriggerGrabber = []() {Grabber.set_value(false);};
	auto moveTransportIn = []() {Transport.move_relative(-270, 100);};
	auto raiseArm = []() {Arm.move_relative(380, 100);};
	auto grabMogo = []() {MobileGoalManipulator.set_value(true);};
	auto dropMogo = []() {MobileGoalManipulator.set_value(false);};

	// Spins Input to intake the first blue ring
	InputMotor.move(-128);

	// Moves to the contested Mogo
	PIDMover({17.75, -48}, false, {raiseArm,triggerGrabber}, {1,30});

	// 
	moveTransportIn();

	PIDMover({28.75, -43.75}, true);

	Grabber.set_value(false);

	pros::delay(125);

	Arm.move_relative(-380, 100);

	PIDTurner(85, 1);

	PIDMover({11, -44.75}, true);

	MobileGoalManipulator.set_value(true);

	PIDMover({24, -42.5}, false, {moveTransportIn,dropMogo}, {1,8});

	PIDTurner(180, 2);

	PIDMover({24, -30.5}, true);

	MobileGoalManipulator.set_value(true);

	Intake.move(-128);

	PIDTurner(findHeadingOfLine(universalCurrentLocation, {65,-72}), 1);

	PIDMover({55, -55.5}, false);

	/*
	//InputMotor.move(-128);
	//PIDMover({78, -48}, true);
	PIDTurner(5, 2);

	Transport.move(-65);
	//PIDMover({48, -48}, false);
	//pros::delay(1000);
	//PIDMover({-48, -48}, true);

	pros::delay(500);
	*/
}


void redGoalside() {

	double motorStartPoint = 0;

	auto triggerGrabber = []() {Grabber.set_value(true);};
	auto moveTransportIn = []() {Transport.move_relative(-540, 200);};

	InputMotor.move(-128);

	pros::Task toMoGo = pros::Task([triggerGrabber] () {PIDMover({10, -49.5}, false, {triggerGrabber}, {27});});
	pros::delay(1250);
	toMoGo.remove();
	AllWheels.brake();

	moveTransportIn();

	PIDMover({21.5, -48.375}, true);

	//PIDTurner(198, 1);
}


void redRingside() {

	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};

	// starts the autonomous by raising the arm and moving to the first Ring
	Arm.move_relative(180, 200);
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
	PIDMover({-58.5, 17}, true);

	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-53.5, 0}), 2);
	PIDMover({-53.5, 0});

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(92, 1);
	AllWheels.move_relative(-365, 100);
	pros::delay(500);
	Transport.move_relative(-285, 200);
	pros::delay(500);

	// moves forward from the Alliance Stake
	PIDMover({-53.75, 0});
	
	// moves to MoGo
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-70.25, -13.25}), 2);
	PIDMover({-28, 21.75}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-180, 200);
	PIDMover({-20.5, 27.25}, true, {gripMoGoM}, {5});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-24, 40}), 2);
	Intake.move(-128);
	PIDMover({-24, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-11.75, 8.5}), 2);
	InputMotor.brake();
	PIDMover({-11.75, 8.5});
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