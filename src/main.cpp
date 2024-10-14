#include "init.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

	pros::lcd::initialize();

	colorSense.set_led_pwm(100);
	Rotational.set_position(0);
	RotationalTurn.set_position(0);
	Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

	Grabber.set_value(false);


	while (true) {
		pros::lcd::print(3, "x = %f", universalCurrentLocation.x);
		pros::lcd::print(4, "y = %f", universalCurrentLocation.y);
	}
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

	autoSelector_task_ptr = new pros::Task(autonSelect);

	while (true) {
		if (globalAuton == true) {
			switch (autonnumber) {
				case 1: //Blue Mogo
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-55, 12}, 148);
					break;
				case 2:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {54.5, 13.125}, 208);
					break;
				case -1:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {54.5, 13.125}, 208);
					break;
				case -2: //Red Ring
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-55, 12}, 148);
					break;
				case 3:
				case -3: //Test (?)
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-48, -48}, 90);
					break;
			}
		} else {
			switch (autonnumber) {
				case 1://Blue Mogo
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {50, -36}, 251);
					break;
				case -1:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-49.25, -60.325}, 69);
					break;
			}
		}
	}
	pros::delay(10);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// disables the auto selector
	if (autoSelector_task_ptr != NULL) {
		autoSelector_task_ptr->remove();
	}

	// starts the system that fixes the turning tracking wheel's heading
	pros::Task updateRotational = pros::Task(bindTurnTrackingWheelHeading);

	// starts the coordinate updating system
	coordinateUpdater_task_ptr = new pros::Task(updateCoordinateLoop);

	// autonomous setup
	colorSense.set_led_pwm(100);

	AllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);

	Intake.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Intake.tare_position();

	Arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Arm.tare_position();

	Kalman1.startFilter();
	Kalman2.startFilter();

	pros::Task* autoEjecter_task_ptr = new pros::Task(autoEject);



	
 
	if (globalAuton == true) {
		switch (autonnumber) {
			case 1:
				globalBlueGoal();
				break;
			case 2:
				globalBlueRing();
				break;
			case -1:
				globalRedGoal();
				break;
			case -2:
				globalRedRing();
				break;
		}
	} else {
		switch (autonnumber) {
			case 1:
				blueGoalside();
				break;
			case -1:
				redGoalside();
				break;
			case 3:
			case -3:
				autoTest();
				break;
		}
	}

	// ending commands
	Master.print(2, 0, "Done");

	pros::delay(1000);
	AllWheels.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

	Grabber.set_value(false);

}




/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
Master.rumble(new char('-'));
	if (coordinateUpdater_task_ptr != NULL) {
		coordinateUpdater_task_ptr->remove();
	}
	if (rotationalBinder_task_ptr != NULL) {
		rotationalBinder_task_ptr->remove();
	}

	ArmPiston.set_value(true);
	// ends the Kalman Filters from autonomous
	Kalman1.endFilter();
	Kalman2.endFilter();

	// resets all pistons
	Grabber.set_value(false);
	
	AllWheels.move_velocity(1000);
	AllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);
	AllWheels.set_brake_modes(MOTOR_BRAKE_COAST);


	// starts the redirect and eject as side tasks
	pros::Task redirectOn(redirect);
	pros::Task ejectOn(eject);

	while (true) {
//Master.print(0, 0, "x = %f", universalCurrentLocation.x);
//Master.print(1, 0, "y = %f", universalCurrentLocation.y);
	//Drivetrain
    	drvtrFB = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    	drvtrLR = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if ((abs(drvtrFB) > drvtrDZ) || (abs(drvtrLR) > drvtrDZ)) {
      		// ^^ Checks to see if either joystick has moved out of the deadzone
			RightWheels.move((drvtrFB-(drvtrLR)));
      		LeftWheels.move((drvtrFB+(drvtrLR)));	
    	} else {
			RightWheels.brake();
      		LeftWheels.brake();
    	}  

	// Intake Conveyor (Transport) and Input
		if(Master.get_digital(DIGITAL_DOWN)){
			Intake.move(128);
		}

	// Input only
		if (Master.get_digital(DIGITAL_L1)) 
		{
			InputMotor.move(-128);
		}
		/* else if(Master.get_digital(DIGITAL_X))
		{
			InputMotor.move(128);
		} */
		else if ((Master.get_digital(DIGITAL_RIGHT) == false)
				  && Master.get_digital(DIGITAL_DOWN) == false && Master.get_digital(DIGITAL_L1) == false) {
			InputMotor.brake();
		}
		
	// Transport braking
		if ((Master.get_digital(DIGITAL_RIGHT) == false) 
			&& (Master.get_digital(DIGITAL_DOWN) == false) && (Master.get_digital(DIGITAL_L2) == false)
			&& (Master.get_digital(DIGITAL_R2) == false) && (Master.get_digital(DIGITAL_LEFT) == false)) {
			Transport.brake();
		}

	
	// Arm (Motor)
		if (Master.get_digital(DIGITAL_Y)) {
			Arm.move(128);
		} else if (Master.get_digital(DIGITAL_B)) {
			Arm.move(-128);
		}
		else {
			Arm.brake();
		}

	//Mobile Goal Manipulator
		// ↓↓ Pressing the R1 Button toggles between modes
		if(Master.get_digital(DIGITAL_R1)){

			// ↓↓ If the manipulator is open, activate this code
			if (MobileGoalManipulator.get_value() == false) {

				// ↓↓ Closes the manipulator to grab an object
				MobileGoalManipulator.set_value(true);

			// ↓↓ If the manipulator is closed, activate this code
			} else if (MobileGoalManipulator.get_value() == true){

				// ↓↓ Opens the manipulator to grab an object
				MobileGoalManipulator.set_value(false);
			}

			// Sets a condition to exit the code that is the 
			// opposite of the condition to enter, preventing
			// us from looping through the code repeatedly ↓↓
			waitUntil(Master.get_digital(DIGITAL_R1) == false);
		}


	// Arm Piston
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			if (ArmPiston.get_value() == false) {
				ArmPiston.set_value(true);
			}
			else {
				ArmPiston.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_UP) == false);
		}

	// Grabber
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			if (Grabber.get_value() == false) {
				Grabber.set_value(true);
			}
			else {
				Grabber.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_L2) == false);
		}

	// Input Piston
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			if (InputPiston.get_value() == false) {
				InputPiston.set_value(true);
			}
			else {
				InputPiston.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_A) == false);
		}
	// Hang
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			if (Hang.get_value() == false) {
				Hang.set_value(true);
			}
			else {
				Hang.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_X) == false);
		}

	// end-of-cycle delay
	pros::delay(10);
	}
}
