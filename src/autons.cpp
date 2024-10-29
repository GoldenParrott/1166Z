#include "init.h"
//hif
void globalBlueRing() {

	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto stopTransport = []() {Transport.brake();};

	// starts the autonomous by raising the arm and moving to the first Ring
	Arm.move_relative(180, 200);
	InputPiston.set_value(true);
	pros::delay(100);
	AllWheels.move_relative(230, 100);
	pros::delay(400);

	// picks up the first Ring
	InputMotor.move(-128);
	InputPiston.set_value(false);
	pros::delay(600);
	InputMotor.brake();
	Transport.move_relative(-210, 200);

	// turns to the Rings next to the Alliance Stake and moves to them
	pros::delay(500);
	InputMotor.move(128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {12, 72}), 2);
	PIDMover({54.25, 0.75}, true);

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(270, 1);
	AllWheels.move_relative(-250,100);
	pros::delay(552);
	Transport.move(-128);
	pros::delay(260);
	Transport.brake();


	// moves forward from the Alliance Stake
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(320, 200);
	waitUntil(posFN() >= initialPos + 320);
	
	// moves to MoGo
	PIDTurner((findHeadingOfLine(universalCurrentLocation, {38, 24}) - 180), 1);
	PIDMover({32, 17.25}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-160, 200);
	PIDMover({21, 27.75}, true, {gripMoGoM}, {5.75});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {28, 40}), 1);
	Intake.move(-128);
	PIDMover({24, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {18, 11}), 1);
	InputMotor.brake();
	Transport.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	PIDMover({14, 13}, false, {stopTransport}, {36});

}


void globalBlueGoal() {

	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};

	// starts the autonomous by raising the arm and moving to the first Ring
	Arm.move_relative(180, 200);
	InputPiston.set_value(true);
	pros::delay(100);
	AllWheels.move_relative(230, 100);
	pros::delay(400);

	// picks up the first Ring
	InputMotor.move(-128);
	InputPiston.set_value(false);
	pros::delay(600);
	InputMotor.brake();
	Transport.move_relative(-210, 200);

	// turns to the Rings next to the Alliance Stake and moves to them
	pros::delay(500);
	InputMotor.move(128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-12, 72}), 1);
	PIDMover({-54.25, 0}, true);

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(90, 2);
	AllWheels.move_relative(-200,100);
	pros::delay(500);
	Transport.move(-128);
	pros::delay(260);
	Transport.brake();


	// moves forward from the Alliance Stake
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(360, 200);
	waitUntil(posFN() >= initialPos + 360);
	
	// moves to MoGo
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-70.25, -15.25}), 2);
	PIDMover({-31.5, 19.75}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-160, 200);
	PIDMover({-20.5, 27.25}, true, {gripMoGoM}, {5.75});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-25, 40}), 2);
	Intake.move(-128);
	PIDMover({-25, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-12, 11}), 2);
	InputMotor.brake();
	PIDMover({-12, 11});

}


void globalRedGoal() {

	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	auto stopTransport = []() {Transport.brake();};

	// starts the autonomous by raising the arm and moving to the first Ring
	Arm.move_relative(180, 200);
	InputPiston.set_value(true);
	pros::delay(100);
	AllWheels.move_relative(230, 100);
	pros::delay(400);

	// picks up the first Ring
	InputMotor.move(-128);
	InputPiston.set_value(false);
	pros::delay(600);
	InputMotor.brake();
	Transport.move_relative(-210, 200);

	// turns to the Rings next to the Alliance Stake and moves to them
	pros::delay(500);
	InputMotor.move(128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {12, 72}), 2);
	PIDMover({54.25, 0.75}, true);

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(270, 1);
	AllWheels.move_relative(-250,100);
	pros::delay(552);
	Transport.move(-128);
	pros::delay(260);
	Transport.brake();


	// moves forward from the Alliance Stake
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(320, 200);
	waitUntil(posFN() >= initialPos + 320);
	
	// moves to MoGo
	PIDTurner((findHeadingOfLine(universalCurrentLocation, {38, 24}) - 180), 1);
	PIDMover({32, 17.25}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-160, 200);
	PIDMover({21, 27.75}, true, {gripMoGoM}, {5.75});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {28, 40}), 1);
	Intake.move(-128);
	PIDMover({24, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {18, 11}), 1);
	InputMotor.brake();
	Transport.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	PIDMover({14, 13}, false, {stopTransport}, {36});
}


void globalRedRing() {

	// mid-PID actions
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};

	// starts the autonomous by raising the arm and moving to the first Ring
	Arm.move_relative(180, 200);
	InputPiston.set_value(true);
	pros::delay(100);
	AllWheels.move_relative(230, 100);
	pros::delay(400);

	// picks up the first Ring
	InputMotor.move(-128);
	InputPiston.set_value(false);
	pros::delay(600);
	InputMotor.brake();
	Transport.move_relative(-210, 200);

	// turns to the Rings next to the Alliance Stake and moves to them
	pros::delay(500);
	InputMotor.move(128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-12, 72}), 1);
	PIDMover({-54.25, 0.875}, true);

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(90, 2);
	AllWheels.move_relative(-200,100);
	pros::delay(500);
	Transport.move(-128);
	pros::delay(260);
	Transport.brake();


	// moves forward from the Alliance Stake
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(360, 200);
	waitUntil(posFN() >= initialPos + 360);
	
	// moves to MoGo
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-70.25, -15.25}), 2);
	PIDMover({-31.5, 19.75}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-160, 200);
	PIDMover({-20, 30.25}, true, {gripMoGoM}, {5.75});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-25, 40}), 2);
	Intake.move(-128);
	PIDMover({-25, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-12, 11}), 2);
	InputMotor.brake();
	PIDMover({-12, 11});
	
}


void redGoalside() {

	// mid-PID actions
	auto triggerGrabber = []() {Grabber.set_value(true);};
	auto untriggerGrabber = []() {Grabber.set_value(false);};
	auto moveTransportIn = []() {Transport.move_relative(-200, 100);};
	auto raiseArm = []() {Arm.move_relative(380, 100);};
	auto grabMoGo = []() {MobileGoalManipulator.set_value(true);};
	auto dropMoGo = []() {MobileGoalManipulator.set_value(false);};
	auto stopIntake = []() {Intake.brake();};

	// Moves to the middle MoGo and intakes the Ring along the way (with time cutoff)
	InputMotor.move(-128);
	pros::Task toMoGo = pros::Task([raiseArm, moveTransportIn] () {PIDMover({-17.375, -48.125}, false, {raiseArm, moveTransportIn}, {1, 33});});
	pros::delay(925);
	toMoGo.remove();
	AllWheels.brake();
	Grabber.set_value(true);
	pros::delay(200);

	// intakes the first Ring into the robot, moves backward with the MoGo yoinked, then ungrabs the MoGo
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(-750, 200);
	waitUntil(posFN() <= initialPos - 750);
	Grabber.set_value(false);

	// turns around and grips the MoGo (with time cutoff)
	pros::delay(125);
	PIDTurner(265, 1);
	toMoGo = pros::Task([raiseArm, moveTransportIn] () {PIDMover({-21, -42.75}, true);});
	pros::delay(575);
	toMoGo.remove();
	AllWheels.brake();
	MobileGoalManipulator.set_value(true);

	// scores the first Ring on the MoGo, then drops it
	pros::delay(300);
	Transport.move_relative(-450, 200);
	pros::delay(650);
	Transport.brake();
	MobileGoalManipulator.set_value(false);

	// turns around, then moves to the other MoGo and grabs it
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-24, -24}) - 180, 1);
	PIDMover({-24, -24}, true);
	AllWheels.move(-128);
	pros::delay(180);
	MobileGoalManipulator.set_value(true);
	pros::delay(20);
	AllWheels.brake();

	// starts intaking and moves to the Corner
	Intake.move(-128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-52, -54}), 2);
	PIDMover({-52, -54}, false);
	pros::delay(300);

	// moves back and forth in the corner to get the bottom Ring
	Intake.move(-128);
	// move in
	AllWheels.move_relative(600, 125);
	pros::delay(1000);
	// back up
	AllWheels.move(-128);
	pros::delay(300);
	AllWheels.brake();

	// turns to and moves to the Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-17, -10}), 2);
	InputMotor.move(128);
	// Arm.move_relative(-380, 200);
	// PIDMover({-17, -10});

}


void blueGoalside() {

	// mid-PID actions
	auto triggerGrabber = []() {Grabber.set_value(true);};
	auto untriggerGrabber = []() {Grabber.set_value(false);};
	auto moveTransportIn = []() {Transport.move_relative(-200, 100);};
	auto raiseArm = []() {Arm.move_relative(380, 100);};
	auto grabMoGo = []() {MobileGoalManipulator.set_value(true);};
	auto dropMoGo = []() {MobileGoalManipulator.set_value(false);};
	auto stopIntake = []() {Intake.brake();};

	// Moves to the middle MoGo and intakes the Ring along the way (with time cutoff)
	InputMotor.move(-128);
	pros::Task toMoGo = pros::Task([raiseArm, moveTransportIn] () {PIDMover({17.375, -48.125}, false, {raiseArm, moveTransportIn}, {1, 33});});
	pros::delay(950);
	toMoGo.remove();
	AllWheels.brake();
	Grabber.set_value(true);
	pros::delay(200);

	// intakes the first Ring into the robot, moves backward with the MoGo yoinked, then ungrabs the MoGo
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(-750, 200);
	waitUntil(posFN() <= initialPos - 750);
	Grabber.set_value(false);

	// lowers the arm a little, turns around, and grips the MoGo (with time cutoff)
	pros::delay(125);
	PIDTurner(89, 1);
	toMoGo = pros::Task([raiseArm, moveTransportIn] () {PIDMover({19, -42.75}, true);});
	pros::delay(800);
	toMoGo.remove();
	AllWheels.brake();
	MobileGoalManipulator.set_value(true);

	// scores the first Ring on the MoGo, then drops it
	pros::delay(300);
	Transport.move_relative(-400, 200);
	pros::delay(550);
	Transport.brake();
	MobileGoalManipulator.set_value(false);

	// turns around, then moves to the other MoGo and grabs it
	PIDTurner(193, 2);
	PIDMover({24, -31}, true);
	MobileGoalManipulator.set_value(true);

	// starts intaking and moves to the Corner
	Intake.move(-128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {69.5, -72}), 1);
	PIDMover({53.5, -56}, false);
	pros::delay(300);

	// moves back and forth in the corner to get the bottom Ring
	Intake.move(-128);
	// move in
	AllWheels.move_relative(600, 125);
	pros::delay(1000);
	// back up
	AllWheels.move(-100);
	pros::delay(300);
	AllWheels.brake();

	// turns to and moves to the Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {0, 0}), 2);
	InputMotor.move(128);
	// Arm.move_relative(-380, 200);
	// PIDMover({13.75, -14.25});

}


void autoSkills() {
// QUADRANT 1
	// sets the time from initialization that the code starts at
	int startTime = pros::millis();

	// mid-PID actions
	auto gripMoGoM = []() {MobileGoalManipulator.set_value(true);};
	
	// scores on the Alliance Stake
	double initialPos = Transport.get_position();
	Transport.move_relative(-340, 200);
	waitUntil((Transport.get_position() <= initialPos - 340) || ((pros::millis() - startTime) / 1000 >= 0.7));
	Transport.move_relative(100, 200);
	pros::delay(300);
	
	// moves forward from the Alliance Stake
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	initialPos = posFN();
	AllWheels.move_relative(530, 200);
	waitUntil(posFN() >= initialPos + 530);

	// turns to a MoGo and moves to it, then grabs it
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-50, -19}) - 180, 1);
	PIDMover({-50, -19.5}, true, {gripMoGoM}, {19});

	// turns and moves to a Ring, then grabs it
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-26, -24}), 2);
	Intake.move(-128);
	Arm.move_relative(380, 200);
	PIDMover({-26, -24});

	// turns and moves to the Ring on the line, then grabs it
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-8, -48}), 2);
	PIDMover({-8, -48});

	// turns and moves to the next three Rings in a line, automatically grabbing them along the way
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-57, -42}), 2);
	PIDMover({-36, -48});
	PIDMover({-57, -42});
	pros::delay(500);

	// turns and moves to the final Ring in this quadrant, then grabs it automatically
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-49.5, -55.5}), 1);
	PIDMover({-49.5, -55.5});

	// turns to the Corner and places the Mobile Goal there
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-58, -57.5}) + 180, 1);
	PIDMover({-58, -57.5}, true);
	MobileGoalManipulator.set_value(false);
	Transport.move_relative(100, 200);
	pros::delay(200);

// QUADRANT 2
		// moves forward from the Corner
		initialPos = posFN();
		AllWheels.move_relative(200, 200);
		waitUntil(posFN() >= initialPos + 200);

		// turns to face the MoGo on the opposite quadrant, then moves to it and grabs it
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-48, 12}) - 180, 2);
		Transport.move(128);
		PIDMover({-48, 12}, true);
		Transport.brake();
		AllWheels.move(-80);
		pros::delay(500);
		MobileGoalManipulator.set_value(true);
		pros::delay(20);
		AllWheels.brake();

		// turns and moves to a Ring, then grabs it
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-26, 16}), 1);
		Intake.move(-128);
		PIDMover({-26, 24});

		// turns and moves to the Ring on the line, then grabs it
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-5, 48}), 1);
		PIDMover({-5, 48});

		// turns and moves to the next three Rings in a line, automatically grabbing them along the way
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-57, 48}), 1);
		PIDMover({-38, 58});
		PIDMover({-57, 48});
		pros::delay(500);

		// turns and moves to the final Ring in this quadrant, then grabs it automatically
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-49.5, 55.5}), 2);
		PIDMover({-49.5, 55.5});

		// turns to the Corner and places the Mobile Goal there
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-54, 59}) - 180, 2);
		PIDMover({-54, 59}, true);
		Transport.move_relative(-100, 200);
		MobileGoalManipulator.set_value(false);
		pros::delay(200);

		// moves forward from the Corner
		initialPos = posFN();
		AllWheels.move_relative(200, 200);
		waitUntil(posFN() >= initialPos + 200);

// QUADRANT 3
			// hi
			// hello
			// sup
			// Drives to quadrant 3 to get the Ring at (24,48)
			InputMotor.move(-128);
			PIDTurner(findHeadingOfLine(universalCurrentLocation, {13.5, 45}), 1);
			PIDMover({13.5, 45});

			// Drives to get the Ring at (24,24)
			Transport.move_relative(-480,100);
			PIDTurner(findHeadingOfLine(universalCurrentLocation, {19, 19}), 2);
			PIDMover({19, 19});
			Transport.move_relative(-300,100);

			// Move and grab the Goal at (48,0)
			PIDTurner(findHeadingOfLine(universalCurrentLocation, {28, 4}) - 180, 1);
			PIDMover({28, 4}, true);
			AllWheels.move(-80);
			pros::delay(400);
			MobileGoalManipulator.set_value(true);
			pros::delay(100);
			AllWheels.brake();
			Transport.move(-128);

			// Move to grab the middle Ring in the corner
			PIDTurner(findHeadingOfLine(universalCurrentLocation, {38, 36}), 2);
			PIDMover({38, 36});

			PIDTurner(findHeadingOfLine(universalCurrentLocation, {49, 36}), 2);
			PIDMover({49, 36});

			// turns and moves to Corner
			PIDTurner(findHeadingOfLine(universalCurrentLocation, {72, 45}) - 180, 2);
			AllWheels.move(-64);
			pros::delay(600);
			AllWheels.brake();
			MobileGoalManipulator.set_value(true);

			// moves forward from the Corner
			initialPos = posFN();
			AllWheels.move_relative(200, 200);
			waitUntil(posFN() >= initialPos + 200);

			
		


}


void autoTest()
{
	PIDMover({0, 0}, true, {[] () {Master.print(0, 0, "hi");}}, {24});
	pros::delay(200);
	PIDMover({48, 48}, true, {[] () {Master.print(0, 0, "ho");}}, {24});
}