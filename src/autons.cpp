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
	PIDTurner(279, 1);

	// Moves to the Alliance Stake
	AllWheels.move(128);
	pros::delay(275);
	AllWheels.brake();

	InputMotor.brake();

	// Scores on the Alliance Stake
	Transport.tare_position();
	Transport.move(128);
	waitUntil(Transport.get_position() >= 2400);
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
	pros::delay(100);
	PIDTurner(165, 1);
	Transport.move(-128);
	PIDMover(3);
	PIDMover(-35, gripMoGoM, -33);

	// Moves to the Ladder to contact it for AWP
	PIDTurner(90, 1);
	PIDMover(6.5);
	PIDMover(3);


	/*
	// Intakes the two Rings in the center of the field\

	// first Ring
	PIDTurner(100, 1);
	PIDMover(7.5);
	pros::delay(250);
	PIDMover(-7.5);
	// second Ring
	PIDTurner(65, 1);
	PIDMover(7.5);
	pros::delay(250);
	PIDMover(-7.5);
	*/
}





void blueGoalside() {
    // lambda functions for use during PID movements
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto activateGrabber = []() {GrabPiston.set_value(true);};
	auto doAFlip = []() {MobileGoalManipulator.set_value(false);};
	auto transportThenGrip = []() {pros::Task transportThenGrip_task(transportThenGripTASK);};



	//drops the input
	Transport.move_relative(600, 200);


	//Starts spinning the Intake
	InputMotor.move(-128);

	// moves toward the second Ring and intakes it after, delaying to give the robot time to fully intake the second Ring
	PIDMover(43, activateGrabber, 41);
	pros::delay(250);
	Transport.move_relative(-1000,200);


	//Moves away from the middle line
	PIDMover(-34);
	
	// Lets go of Mobile Goal
	GrabPiston.set_value(false);
	pros::delay(62);

	//Turns to pick up Mobile Goal 
	PIDTurner(162, 2);

	//Moves to the Mobile Goal to pick it up
	PIDMover(-23);
	PIDMover(-9, gripMoGoM, -6);
	pros::delay(250);

	// Puts the Rings on the Mobile Goal
	Transport.move(-128);

	
	// Turns toward opposite Ring, moves to it, and intakes it to ensure alignment with the Corner
	PIDTurner(225, 2);
	PIDMover(21); // needs to be checked

	// Turns toward Corner Rings, moves to them, and intakes them
	PIDTurner(247, 2);
	pros::delay(500);
	PIDMover(6.5);
	pros::delay(500);
	// moves the robot back and forth some to guarantee that the third Ring will get in
	pros::delay(200);
	PIDMover(-4);
	// moves the third Ring into the end of the intake
	Transport.brake();
	Transport.move_relative(-3000, 200);


	// maneuvers the robot to the other Mobile Goal, droppng off the first one in the process
	PIDMover(-18);
	
	PIDTurner(315, 2);
	PIDMover(-36, doAFlip, -32);
	PIDMover(12);
	PIDTurner(260, 1);

	// Picks up Mobile Goal
	PIDMover(12, gripMoGoM, 10);
	PIDTurner(120, 1);
	Transport.move(128);
	
}


void redGoalside() {}


void redRingside() {}