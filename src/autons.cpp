#include "init.h"

void blueRingside() {

	// lambda functions for use during PID movements
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto transportIn = []() {Transport.move(-128);};

    // turns on the intake to push the intake down
	Transport.tare_position();
	Transport.move_relative(660, 200);
	waitUntil(Transport.get_position() >= 660);
	InputMotor.move(-128);
	

	// intakes the second Ring from the top of the stack and outtakes the bottom Ring
	PIDMover(3);
	Transport.tare_position();
	Transport.move_relative(-720, 200);
	waitUntil(Transport.get_position() <= -720);

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
	Transport.tare_position();
	Transport.move(128);
	InputMotor.move(128);
	// First Ring
	int overRide = 0;
	waitUntil(Transport.get_position() >= 1250);
	Transport.brake();
	// Push back in
	AllWheels.move(128);
	pros::delay(50);
	AllWheels.brake();
	// Second Ring
	Transport.move(128);
	while (!(Transport.get_position() >= 2500)) {
		overRide += 50; 
		if (overRide >= 1250) {Transport.move_relative(-350, 200); break;}
		pros::delay(50);
	}
	Transport.brake();

	// Maneuvers to grab the next Ring and drops the arm along the way
	PIDMover(-3);
	PIDTurner(143, 1);
	InputMotor.move(-128);
	pros::Task lowerArm_task(lowerArm);
	PIDMover(52);
	pros::delay(250);
	

	// Intakes the Ring across from the Mobile Goal
	PIDTurner(190, 2);
	pros::delay(150);
	PIDTurner(165, 1);
	Transport.move(-128);
	PIDMover(3);

	// Moves to the Mobile Goal and grips it
	PIDMover(-31, {gripMoGoM}, {-29});
	Transport.brake();

	// Maneuvers to the Ladder to contact it for AWP
	PIDTurner(105, 1);
	Transport.move(-128);
	PIDMover(9.5);

	// Sets up the Input to contact the Ladder
	Transport.brake();
	InputPiston.set_value(true);
	pros::delay(250);
	InputPiston.set_value(false);

	// Moves into the Ladder and contacts it
	AllAllWheels.move(100); 
	pros::delay(300); 
	AllAllWheels.brake();
}





void blueGoalside() {
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
	PIDMover(43, {activateGrabber}, {41});
	pros::delay(250);
	Transport.move_relative(-900,200);


	//Moves away from the middle line
	PIDMover(-34);
	
	// Lets go of Mobile Goal
	GrabPiston.set_value(false);
	pros::delay(200);

	//Turns to pick up Mobile Goal 
	PIDTurner(120, 2);
	pros::delay(100);
	PIDTurner(170, 2);

	//Moves to the Mobile Goal to pick it up
	PIDMover(-25, {gripMoGoM}, {-20});
	pros::delay(250);


	
	// Turns toward opposite Ring, moves to it, and intakes it to ensure alignment with the Corner
	PIDTurner(185, 2);
	// Puts the Rings on the Mobile Goal
	Transport.move(-128);
	InputMotor.move(128);
	PIDMover(33); // needs to be checked
	PIDMover(-3);

	// Turns toward Corner Rings, moves to them, and intakes them
	PIDTurner(240, 2);
	InputMotor.move(-128);
	// Moves back and forth to intake the Ring well
	AllAllWheels.move(51);
	pros::delay(1400);
	AllAllWheels.brake();
	PIDMover(-8);
	PIDMover(3);
	pros::Task blockBlueRing_task(blockBlueRing); // stops the Transport when the third Ring is detected at the end
	pros::delay(500);
	
	// Moves to the other Mobile Goal and drops the first
	PIDMover(-3);
	PIDTurner(290, 2);
	PIDMover(-32, {reverseInput}, {-10});
	MobileGoalManipulator.set_value(false);

	// Goes to pick up the other Mobile Goal
	PIDMover(6);
	pros::delay(100);
	PIDTurner(235, 1);
	PIDMover(-31, {gripMoGoM}, {-29});

	// Maneuvers to the Ladder and scores the final Ring
	PIDTurner(150, 1);
	Transport.move(-128);
	AllAllWheels.move(51);
	pros::delay(2000);
	AllAllWheels.brake();


	/*
	Transport.brake();
	GrabPiston.set_value(true);
	InputPiston.set_value(true);
	pros::delay(500);
	PIDMover(9); // aligns for the sweep
	pros::delay(500);
	PIDTurner(120, 1); // the sweep

	// re-enables the transport after removing the setup for the sweep
	InputPiston.set_value(false);
	pros::delay(62);
	Transport.move(-128);
	

	// moves the robot away from the wall and ensures that the third Ring is intaked
	PIDMover(12);
	MobileGoalManipulator.set_value(false);
	
	// moves to the second Mobile Goal and grips it
	PIDTurner(220, 2);
	PIDMover(24);
	PIDMover(-7, gripMoGoM, -5);
	Transport.move_relative(1000, 200);
	pros::delay(125);
	PIDMover(-7, transportToLadder, -2);

	*/
}


void redGoalside() {}


void redRingside() {}