#include "init.h"

void blueRingside() {
	Inertial.tare_heading();

	// lambda functions for use during PID movements
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto transportIn = []() {Transport.move(-128);};

    // turns on the intake to push the intake down
	Transport.tare_position();
	Transport.move_relative(500, 200);
	waitUntil(Transport.get_position() >= 500);
	InputMotor.move(-128);
	

	// intakes the second Ring from the top of the stack and outtakes the bottom Ring
	PIDMover(3);
	Transport.tare_position();
	Transport.move_relative(-400, 200);
	waitUntil(Transport.get_position() <= -400);

	InputMotor.move(128);

	// Backs up and raises the arm
	PIDMover(-8);
	pros::Task raiseArm_task(raiseArm);


	// Maneuvers to the Alliance Stake
	PIDTurner(346.5, 1);
	PIDMover(24);
	PIDMover(-2);
	PIDTurner(278, 1);

	// Moves to the Alliance Stake
	AllWheels.move(128);
	pros::delay(300);
	AllWheels.brake();

	InputMotor.brake();

	// Scores on the Alliance Stake
	InputMotor.move(128);
	int overRide;
	// First Ring
	Transport.tare_position();
	Transport.move(128);
	overRide = 0;
	while (!(Transport.get_position() >= 1250)) {
		overRide += 50;
		if (overRide >= 1250) {
			// backs robot up to unblock Ring
			AllWheels.move(-128);
			pros::delay(75);
			AllWheels.brake();
			break;
		}
		pros::delay(50);
	}
	Transport.brake();
	// Push back in
	AllWheels.move(128);
	pros::delay(125);
	AllWheels.brake();
	// Second Ring
	Transport.move(128);
	overRide = 0;
	while (!(Transport.get_position() >= 2500)) {
		overRide += 50;
		if (overRide >= 1250) { 
			break;
		}
		pros::delay(50);
	}
	Transport.move_relative(-500, 200); 
	pros::delay(250);
	Transport.brake();

	// Maneuvers to grab the next Ring and drops the arm along the way
	PIDMover(-3);
	PIDTurner(130, 1);
	InputMotor.move(-128);
	pros::Task lowerArm_task(lowerArm);
	Transport.move(-128);
	PIDMover(52);
	pros::delay(250);
	

	// Intakes the Ring across from the Mobile Goal
	// (transport is already moving)
	PIDTurner(165, 2);

	// Moves to the Mobile Goal and grips it
	PIDMover(-30, gripMoGoM, -26);
	// Transport.brake();

	// Maneuvers to the Ladder to contact it for AWP
	PIDTurner(90, 1);
	Transport.move(-128);
	pros::delay(1000);
	PIDMover(9.5);

	// Sets up the Input to contact the Ladder
	InputPiston.set_value(true);
	pros::delay(250);
	InputPiston.set_value(false);

	// Moves into the Ladder and contacts it
	AllAllWheels.move(100);
	AllAllWheels.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	pros::delay(500); 
	AllAllWheels.brake();
}





void blueGoalside() {
	Inertial.tare_heading();

    // lambda functions for use during PID movements
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto activateGrabber = []() {GrabPiston.set_value(true);};
	auto doAFlip = []() {MobileGoalManipulator.set_value(false);};
	auto transportThenGrip = []() {pros::Task transportThenGrip_task(transportThenGripTASK);};
	auto stopTransport = []() {Transport.brake();};
	auto transportIn = []() {Transport.move(-128);};
	auto transportToLadder = []() {Transport.move_relative(1000, 200);};
	auto reverseInput = []() {InputMotor.move(128);};


	//drops the input
	Transport.move_relative(600, 200);


	//Starts spinning the Intake
	InputMotor.move(-128);

	// moves toward the second Ring and intakes it after, delaying to give the robot time to fully intake the second Ring
	PIDMover(43, activateGrabber, 41);
	pros::delay(250);
	Transport.move_relative(-900,200);


	//Moves away from the middle line
	PIDMover(-34);
	
	// Lets go of Mobile Goal
	GrabPiston.set_value(false);
	pros::delay(200);

	//Turns to pick up Mobile Goal
	PIDTurner(165, 2);

	//Moves to the Mobile Goal to pick it up
	PIDMover(-25, gripMoGoM, -20);
	pros::delay(250);


	
	// Turns toward opposite Ring, moves to it, and intakes it to ensure alignment with the Corner
	int dir = 0;
	switch (Inertial.get_heading() < 180) {
		case true:
			dir = 1;
			break;
		case false:
			dir = 2;
			break;
	}
	if (Inertial.get_heading() == 180) {
		PIDTurner(180, 2);
	}
	// Puts the Rings on the Mobile Goal
	Transport.move(-128);
	// InputMotor.move(128);
	PIDMover(33);
	PIDMover(-4);

	// Turns toward Corner Rings, moves to them, and intakes them
	PIDTurner(230, 2);
	InputMotor.move(-128);
	// Moves back and forth to intake the Ring well
	AllAllWheels.move(54);
	pros::delay(1500);
	AllAllWheels.brake();
	PIDMover(-8);
	PIDMover(3);
	pros::Task blockBlueRing_task(blockBlueRing); // stops the Transport when the third Ring is detected at the end
	pros::delay(500);
	
	// Moves to the other Mobile Goal and drops the first
	PIDMover(-3);
	PIDTurner(285, 2);
	PIDMover(-32); //, reverseInput, -10);
	MobileGoalManipulator.set_value(false);

	// Goes to pick up the other Mobile Goal
	PIDMover(8);
	pros::delay(100);
	PIDTurner(240, 1);
	PIDMover(-35, gripMoGoM, -31);

	// Maneuvers to the Ladder and scores the final Ring
	PIDTurner(132, 1);
	Transport.move(-128);
	pros::delay(833);
	AllAllWheels.move(54);
	AllAllWheels.set_brake_modes(MOTOR_BRAKE_COAST);
	pros::delay(2000);
	AllAllWheels.brake();
}





void redGoalside() {
	Inertial.tare_heading();

	AllAllWheels.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

	auto activateGrabber = []() {GrabPiston.set_value(true);};
	auto deactivateGrabber = []() {GrabPiston.set_value(false);};
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto gripMoGoMThenTransport = []() {MobileGoalManipulator.set_value(true); Transport.move_relative(-1700, 200);};
	auto transportIn = []() {Transport.move_relative(-900, 200);};
	

	//drops the input
	Transport.move_relative(600, 200);


	//Starts spinning the Intake out to push off the top Ring
	InputMotor.move_relative(1300, 600);

	// moves toward the second Ring and intakes it after, delaying to give the robot time to fully intake the second Ring
	PIDMover(34, activateGrabber, 32);
	InputMotor.move(-128);
	pros::delay(350);

	// turns to bring the MoGo back to the wall
	AllLeftWheels.move(75);
	AllRightWheels.move(-75);
	waitUntil(Inertial.get_heading() > 190 && Inertial.get_heading() < 210);
	AllAllWheels.brake();
	pros::delay(200);

	// releases the MoGo and turns around to grab it
	GrabPiston.set_value(false);
	PIDTurner((Inertial.get_heading() - 10), 1);
	pros::delay(200);
	Transport.move_relative(-900, 200);
	PIDTurner(56, 1);

	// drops the first MoGo off at the back
	PIDMover(3);
	PIDMover(-40, gripMoGoM, -33);
	Transport.move_relative(-700, 200);
	pros::delay(1000);
	MobileGoalManipulator.set_value(false);
	
	// picks up the other MoGo
	PIDMover(17);
	pros::delay(100);
	PIDTurner(110, 2);
	PIDMover(-24, gripMoGoM, -20);

	// Scores the other Ring and touches the Ladder
	PIDMover(10);
	pros::delay(500);
	PIDTurner(20, 1);
	Transport.move(-128);
	pros::delay(500);
	AllAllWheels.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	InputPiston.set_value(true);
	PIDMover(10);
	InputPiston.set_value(false);


}


void redRingside() {
	Inertial.tare_heading();

	// lambda functions for use during PID movements
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto transportIn = []() {Transport.move(-128);};

    // turns on the intake to push the intake down
	Transport.tare_position();
	Transport.move_relative(500, 200);
	waitUntil(Transport.get_position() >= 500);
	InputMotor.move(-128);
	

	// intakes the second Ring from the top of the stack and outtakes the bottom Ring
	PIDMover(3);
	Transport.tare_position();
	Transport.move_relative(-400, 200);
	waitUntil(Transport.get_position() <= -400);

	InputMotor.move(128);

	// Backs up and raises the arm
	PIDMover(-8);
	pros::Task raiseArm_task(raiseArm);


	// Maneuvers to the Alliance Stake
	PIDTurner(13.5, 2);
	PIDMover(24);
	PIDMover(-2);
	PIDTurner(82, 2);

	// Moves to the Alliance Stake
	AllWheels.move(128);
	pros::delay(300);
	AllWheels.brake();

	InputMotor.brake();

	// Scores on the Alliance Stake
	InputMotor.move(128);
	int overRide;
	// First Ring
	Transport.tare_position();
	Transport.move(128);
	overRide = 0;
	while (!(Transport.get_position() >= 1250)) {
		overRide += 50;
		if (overRide >= 1250) {
			// backs robot up to unblock Ring
			AllWheels.move(-128);
			pros::delay(75);
			AllWheels.brake();
			break;
		}
		pros::delay(50);
	}
	Transport.brake();
	// Push back in
	AllWheels.move(128);
	pros::delay(125);
	AllWheels.brake();
	// Second Ring
	Transport.move(128);
	overRide = 0;
	while (!(Transport.get_position() >= 2500)) {
		overRide += 50;
		if (overRide >= 1250) { 
			break;
		}
		pros::delay(50);
	}
	Transport.move_relative(-500, 200); 
	pros::delay(250);
	Transport.brake();

	// Maneuvers to grab the next Ring and drops the arm along the way
	PIDMover(-3);
	PIDTurner(230, 2);
	InputMotor.move(-128);
	pros::Task lowerArm_task(lowerArm);
	Transport.move(-128);
	PIDMover(52);
	pros::delay(250);
	

	// Intakes the Ring across from the Mobile Goal
	// (transport is already moving)
	PIDTurner(195, 1);

	// Moves to the Mobile Goal and grips it
	PIDMover(-30, gripMoGoM, -26);
	// Transport.brake();

	// Maneuvers to the Ladder to contact it for AWP
	PIDTurner(270, 2);
	Transport.move(-128);
	pros::delay(1000);
	PIDMover(9.5);

	// Sets up the Input to contact the Ladder
	InputPiston.set_value(true);
	pros::delay(250);
	InputPiston.set_value(false);

	// Moves into the Ladder and contacts it
	AllAllWheels.move(100);
	AllAllWheels.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	pros::delay(500); 
	AllAllWheels.brake();
}
/*
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
	}
	pros::delay(50);
} */