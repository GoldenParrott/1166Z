#include "init.h"



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

	autonnumber = 1;

	pros::lcd::initialize();

	colorSense.set_led_pwm(100);
	Rotational.set_position(0);
	RotationalTurn.set_position(0);

	pros::Task updateRotational = pros::Task(updateOdomPosition);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

	MobileGoalManipulator.set_value(false);
	Grabber.set_value(false);

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
	pros::screen::touch_callback(autonSwitcher, TOUCH_PRESSED);

	switch (autonnumber) {
		case 1: 
			initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-120, -120}, 90);
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
	}
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

	// starts the coordinate updating system
	pros::Task coordinateUpdater_task = pros::Task(updateCoordinateLoop);

	// autonomous setup
	colorSense.set_led_pwm(100);

	AllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);

	Intake.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Intake.tare_position();

	Arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Arm.tare_position();

	Kalman1.startFilter();
	Kalman2.startFilter();

	
	
	// drawLogo();

switch (autonnumber) {
	case 1:
		blueGoalside();
		break;
	case 2:
		blueRingside();
		break;
	case -1:
		redGoalside();
		break;
	case -2:
		redRingside();
		break;

}

	// ending commands
	//Master.print(0, 0, 'Done');

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


	// initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {0, 0}, 270, 2);
	pros::Task coordinateUpdater_task = pros::Task(updateCoordinateLoop);

	pros::lcd::initialize();
	pros::delay(4000);


	Kalman1.endFilter();
	Kalman2.endFilter();

	// resets all pistons
	Grabber.set_value(false);
	
	AllWheels.move_velocity(1000);
	AllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);
	AllWheels.set_brake_modes(MOTOR_BRAKE_COAST);

	Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	while (true) {

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
		if (Master.get_digital(DIGITAL_RIGHT)){
			Intake.move(-128);
		} else if(Master.get_digital(DIGITAL_DOWN)){
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
		else if ((Master.get_digital(DIGITAL_RIGHT) == false) && (Master.get_digital(DIGITAL_LEFT) == false) 
				  && Master.get_digital(DIGITAL_DOWN) == false && Master.get_digital(DIGITAL_L1) == false) {
			InputMotor.brake();
		}
		
	// Transport only
		if (Master.get_digital(DIGITAL_LEFT)) {
			Transport.move(128);	
		}
		else if ((Master.get_digital(DIGITAL_LEFT) == false) && (Master.get_digital(DIGITAL_RIGHT) == false) 
					&& (Master.get_digital(DIGITAL_DOWN) == false) && (Master.get_digital(DIGITAL_L2) == false)) {
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

// hi :)
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


	// color sensor (redirect)

		// handles the cases for if R2 is being held down
		if (Master.get_digital(DIGITAL_R2)) {
			// case 1: redirect is currently on
			if (redirectOn == true) {
				// case 1a: if the difference between the starting point and the current point
				// 			is greater than 700 (meaning that it has gone all the way), 
				//			turn off the redirect
				if (abs(Transport.get_position() - redirectStartPoint) >= 2100) {
					redirectOn = false;
					redirectStartPoint = 0;
					Transport.brake();
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					Intake.move(80);
				}
			}
			// case 2: redirect is not on, but the color sensor has found a Ring of the proper color
			else if ((colorSense.get_hue() > 200 || autonnumber > 0) || // blue
				     (colorSense.get_hue() < 16 && (colorSense.get_hue() > 10) && (autonnumber < 0))) // red
			{
				// in this case, the redirect is started and the starting point is stored for later
				Intake.move(80);
				redirectOn = true;
				redirectStartPoint = Transport.get_position();
			}
			// case 3: if the redirect is not on and should not be on, 
			//		   then L2 moves the robot forward as normal
			else {
				Intake.move(-80);
			}
		// if L2 is not being pressed, then the redirect is turned off
		} else if (!Master.get_digital(DIGITAL_R2)) {
			redirectOn = false;
			redirectStartPoint = 0;
		}

		Master.print(0, 0, "RD = %d", redirectOn);

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
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			if (Grabber.get_value() == false) {
				Grabber.set_value(true);
			}
			else {
				Grabber.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_A) == false);
		}

	// Input Piston
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			if (InputPiston.get_value() == false) {
				InputPiston.set_value(true);
			}
			else {
				InputPiston.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_L2) == false);
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
	pros::delay(20);
	}
}
