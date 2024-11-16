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
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-22, -26}) - 180, 1);
	PIDMover({-22, -26}, true);
	AllWheels.move(-128);
	pros::delay(200);
	MobileGoalManipulator.set_value(true);
	pros::delay(20);
	AllWheels.brake();

	// starts intaking and moves to the Corner
	Intake.move(-128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-50, -55}), 2);
	PIDMover({-50, -55});
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
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-18, -8}), 2);
	InputMotor.move(128);
	Arm.move_relative(-380, 200);
	PIDMover({-18, -8});

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
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {50, -55}), 1);
	PIDMover({50, -55});
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
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {17, -12}), 2);
	InputMotor.move(128);
	//Arm.move_relative(-380, 200);
	PIDMover({17, -17});

}


void redRingside() {

	// lambda functions for mid-PID movements (and as convenient macros)
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos;

	// raises the arm and scores on the Alliance Stake with it
	Arm.move_relative(100, 200);
	pros::delay(200);
	Arm.move_relative(-100, 200);
	pros::delay(300);
	Arm.move_relative(325, 150);
	pros::delay(125);
	InputMotor.move(128);
	initialPos = posFN();
	AllWheels.move_relative(-90, 500);
	waitUntil(posFN() <= initialPos - 90);
	AllWheels.brake();
	pros::delay(1000);
	initialPos = posFN();
	AllWheels.move_relative(370, 500);
	waitUntil(posFN() >= initialPos + 370);
	AllWheels.brake();
	Arm.move_relative(-130, 200);

	// backs up from the Alliance Stake
	initialPos = posFN();
	AllWheels.move_relative(-360, 500);
	waitUntil(posFN() <= initialPos - 360);
	AllWheels.brake();
	Arm.move_relative(-130, 200);

	// backs into MoGo and grips it
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-30.125, 18.5}) - 180, 2);
	PIDMover({-30.125, 18.5}, true);
	AllWheels.move(-128);
	pros::delay(200);
	MobileGoalManipulator.set_value(true);
	pros::delay(50);
	AllWheels.brake();

	// 
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-24, 42}), 2);
	Intake.move(-128);
	PIDMover({-24, 42});


}


void blueRingside() {

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
	CutoffTurnPID({-49, -19}, true, 1000, 1);
	PIDMover({-49, -19.5}, true, {gripMoGoM}, {18});

	// turns and moves to a Ring, then grabs it
	CutoffTurnPID({-26, -24}, false, 800, 2);
	Intake.move(-128);
	Arm.move_relative(380, 200);
	CutoffPID({-26, -24}, false, 900);

	// turns and moves to the Ring on the line, then grabs it
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-8, -48}), 2);
	CutoffPID({-8, -48}, false, 2200);

	// turns and moves to the next three Rings in a line, automatically grabbing them along the way
	Transport.brake();
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {-57, -45}), 2);
	Transport.move(-128);
	CutoffPID({-36, -49}, false, 1800);
	CutoffPID({-57, -43}, false, 1850);
	pros::delay(500);

	// turns and moves to the final Ring in this quadrant, then grabs it automatically
	Transport.brake();
	CutoffTurnPID({-49.5, -55.5}, false, 1000, 1);
	Transport.move(-128);
	CutoffPID({-49.5, -53.5}, false, 1500);

	// turns to the Corner and places the Mobile Goal there
	CutoffTurnPID({-58, -57.5}, true, 1300, 1);
	CutoffPID({-58, -57.5}, true, 500);
	MobileGoalManipulator.set_value(false);
	Transport.move_relative(200, 200);
	pros::delay(200);



// QUADRANT 2
		// moves forward from the Corner
		initialPos = posFN();
		AllWheels.move_relative(200, 200);
		waitUntil(posFN() >= initialPos + 200);

		// turns to face the MoGo on the opposite quadrant, then moves to it and grabs it
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-44, 10}) - 180, 2);
		Transport.move(128);
		PIDMover({-44, 10}, true);
		Transport.brake();
		AllWheels.move(-80);
		pros::delay(500);
		MobileGoalManipulator.set_value(true);
		pros::delay(20);
		AllWheels.brake();

		// turns and moves to a Ring, then grabs it
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-26, 16}), 1);
		Intake.move(-128);
		CutoffPID({-26, 24}, false, 1500);


		// turns and moves to the Ring on the line, then grabs it
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-5, 48}), 1);
		CutoffPID({-5, 48}, false, 1800);
		

		// turns and moves to the next three Rings in a line, automatically grabbing them along the way
		Transport.brake();
		PIDTurner(findHeadingOfLine(universalCurrentLocation, {-57, 48}), 1);
		Transport.move(-128);
		CutoffPID({-38, 58}, false, 1750);
		CutoffPID({-57, 48}, false, 1800);
		pros::delay(500);

		// turns and moves to the final Ring in this quadrant, then grabs it automatically
		CutoffTurnPID({-49.5, 55.5}, false, 1000, 2);
		CutoffPID({-49.5, 55.5}, false, 1000);

		// turns to the Corner and places the Mobile Goal there
		CutoffTurnPID({-54, 59}, true, 1000, 2);
		CutoffPID({-54, 59}, true, 1000);
		Transport.move_relative(200, 200);
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
			//PIDMover({13.5, 45});
			CutoffPID({13.5, 45}, false, 2250);

			// Drives to get the Ring at (24,24)
			Transport.move_relative(-480,100);
			PIDTurner(findHeadingOfLine(universalCurrentLocation, {19, 19}), 2);
			CutoffPID({19, 19}, false, 2000);
			Transport.move_relative(-300,100);

			// Move and grab the Goal at (48,0)
			PIDTurner(findHeadingOfLine(universalCurrentLocation, {30, 4}) - 180, 1);
			PIDMover({30, 4}, true);
			pros::delay(100);
			AllWheels.move(-40);
			pros::delay(600);
			MobileGoalManipulator.set_value(true);
			pros::delay(100);
			AllWheels.brake();
			Transport.move(-128);

			// Move to grab the middle Ring in the corner
			CutoffTurnPID({38, 36}, false, 1000, 2);
			CutoffPID({38, 36}, false, 1250);

			CutoffTurnPID({70, 15}, false, 1000, 2);
			initialPos = posFN();
			AllWheels.move_relative(450, 600);
			waitUntil(posFN() >= initialPos + 450);

			// turns and moves to Corner
			//CutoffTurnHeadingPID(220, false, 1500, 2);
			PIDTurner(220, 2);
			MobileGoalManipulator.set_value(false);
			AllWheels.move(-64);
			pros::delay(600);
			AllWheels.brake();

			// moves forward from the Corner
			CutoffPID({38, 36}, false, 1000);
			PIDTurner(349, 2);
			AllWheels.move(-128);
			pros::delay(1500);
			AllWheels.move(70);
			pros::delay(750);
			AllWheels.brake();



			
		


}


void autoTest()
{
	PIDMover({0, 0}, true, {[] () {Master.print(0, 0, "hi");}}, {24});
	pros::delay(200);
	PIDMover({48, 48}, true, {[] () {Master.print(0, 0, "ho");}}, {24});
}