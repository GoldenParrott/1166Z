#include "init.h"
//hif
void globalBlueRing() {

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
	Transport.move_relative(-220, 200);

	// turns to the Rings next to the Alliance Stake and moves to them
	pros::delay(500);
	InputMotor.move(128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {12, 72}), 2);
	PIDMover({54.25, 0}, true);

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(270, 1);
	AllWheels.move_relative(-200,100);
	pros::delay(500);
	Transport.move(-128);
	pros::delay(250);
	Transport.brake();


	// moves forward from the Alliance Stake
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(320, 200);
	waitUntil(posFN() >= initialPos + 320);
	
	// moves to MoGo
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {71.5, -24}), 1);
	PIDMover({31, 18.25}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-160, 200);
	PIDMover({21, 27.75}, true, {gripMoGoM}, {5.75});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {28, 40}), 1);
	Intake.move(-128);
	PIDMover({24, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {15, 11}), 1);
	InputMotor.brake();
	Transport.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	PIDMover({14, 13});

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
	pros::delay(250);
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
	Transport.move_relative(-220, 200);

	// turns to the Rings next to the Alliance Stake and moves to them
	pros::delay(500);
	InputMotor.move(128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {12, 72}), 2);
	PIDMover({54.25, 0}, true);

	// turns to face the intake to the Alliance Stake and moves to it, then scores on it
	PIDTurner(270, 1);
	AllWheels.move_relative(-200,100);
	pros::delay(500);
	Transport.move(-128);
	pros::delay(250);
	Transport.brake();


	// moves forward from the Alliance Stake
	auto posFN = []() {return (BackRight.get_position() + BackLeft.get_position() + FrontRight.get_position() + FrontLeft.get_position()) / 4;};
	double initialPos = posFN();
	AllWheels.move_relative(320, 200);
	waitUntil(posFN() >= initialPos + 320);
	
	// moves to MoGo
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {71.5, -24}), 1);
	PIDMover({31, 18.25}, true); // ensures that the robot approaches the MoGo slow enough by splitting it into two movements
	Arm.move_relative(-160, 200);
	PIDMover({21, 27.75}, true, {gripMoGoM}, {5.75});

	// turns to, moves to, and intakes Rings in middle of quadrant
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {28, 40}), 1);
	Intake.move(-128);
	PIDMover({24, 40});

	// turns to, moves to, and touches Ladder
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {15, 11}), 1);
	InputMotor.brake();
	Transport.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	PIDMover({14, 13});
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
	pros::delay(250);
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


void redGoalside() {

	// mid-PID actions
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


void blueGoalside() {

	// mid-PID actions
	auto triggerGrabber = []() {Grabber.set_value(true);};
	auto untriggerGrabber = []() {Grabber.set_value(false);};
	auto moveTransportIn = []() {Transport.move_relative(-210, 100);};
	auto raiseArm = []() {Arm.move_relative(380, 100);};
	auto grabMoGo = []() {MobileGoalManipulator.set_value(true);};
	auto dropMoGo = []() {MobileGoalManipulator.set_value(false);};

	// Moves to the middle MoGo and intakes the Ring along the way
	InputMotor.move(-128);
	pros::Task toMoGo = pros::Task([raiseArm, moveTransportIn] () {PIDMover({17.375, -48.125}, false, {raiseArm, moveTransportIn}, {1, 31});});
	pros::delay(1000);
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

	// lowers the arm a little, turns around, and grips the MoGo
	pros::delay(125);
	//Arm.move_relative(-20, 100);
	PIDTurner(72, 1);
	PIDMover({20.375, -44.5}, true);
	MobileGoalManipulator.set_value(true);

	// scores the first Ring on the MoGo, then drops it
	pros::delay(200);
	Transport.move_relative(-250, 200);
	pros::delay(500);
	Transport.brake();
	MobileGoalManipulator.set_value(false);

	// turns around, then moves to the other MoGo and grabs it
	PIDTurner(180, 2);
	PIDMover({24, -30.5}, true);
	MobileGoalManipulator.set_value(true);

	// starts intaking and moves to the Corner
	Intake.move(-128);
	PIDTurner(findHeadingOfLine(universalCurrentLocation, {55, -56}), 1);

	PIDMover({55, -56}, false);
/*
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
*/
}


void autoTest()
{
	//setup
	// raises the arm and startes the intake moving and props up the input.
	Arm.move_relative(180, 200);
	InputMotor.move(-128);
	InputPiston.set_value(true);
	pros::delay(250);

	// moves tword the stack of rings 
	PIDMover({-24,-48});

	// lowers the input and grabs the top ring
	InputPiston.set_value(false);
	pros::delay(250);
	Transport.move_relative(-300,200);

	//moves back and turns
	PIDMover({-48,-48},true);
	PIDTurner(5, 1 );
	AllWheels.move_relative(-365, 100);
	pros::delay(500);
	Transport.move(-128);
	pros::delay(400);
	Transport.brake();

;}

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