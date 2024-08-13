#include "init.h"



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

	autonnumber = -1;
	IntakePTOPiston.set_value(false);
	if (abs(autonnumber) == 2) {
		IntakePTOPiston.set_value(true);
	}
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
/*
	MobileGoalManipulator.set_value(false);
	InputPiston.set_value(false);
	GrabPiston.set_value(false);
*/
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

	// autonomous setup
	colorSense.set_led_pwm(100);
	AllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Intake.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Intake.tare_position();
	ArmLeft.set_encoder_units(MOTOR_ENCODER_DEGREES);
	ArmLeft.tare_position();

	if (colorSensorOn_task_ptr == NULL) {
		// colorSensorOn_task_ptr = new pros::Task(colorSensorOn, 'Color Eject On');
	}

	
	drawLogo();

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
/*
	GrabPiston.set_value(false);
	Eject.set_value(false);
*/
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

/*
	if (colorSensorOn_task_ptr != NULL) {
		colorSensorOn_task_ptr->remove();
	}

	// resets all pistons
	IntakePTOPiston.set_value(false);
	intakePTOvalue = false;
	Eject.set_value(false);
	GrabPiston.set_value(false);
	
	AllWheels.move_velocity(1000);
	AllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);
	AllWheels.set_brake_modes(MOTOR_BRAKE_COAST);
*/
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
			Intake.move(128);
		} else if(Master.get_digital(DIGITAL_LEFT)){
			Intake.move(-128);
		} else {
			Intake.brake();
		}

	// Claw Arm


		armPosition = abs(ArmLeft.get_position());

		if (intakePTOvalue == true) {
			if (!presettingX && !presettingA) {
				if (Master.get_digital(DIGITAL_DOWN)) {
					Arm.move(-128);
				} else if (Master.get_digital(DIGITAL_UP)) {
					Arm.move(128);
				} else {
					Arm.brake();
				}
			}
		}
/*

		// Arm Presets


		// Neutral Stake Preset
			if (Master.get_digital(DIGITAL_X) && !presettingA) {
				Arm.move(-128);
				presettingX = true;
			} 
		// Alliance Stake Preset
			else if (Master.get_digital(DIGITAL_A) && !presettingX) {
				Arm.move(-128);
				presettingA = true;
			}

		// These stop the preset movements in their own separate check to prevent the code from blocking other code
			if (presettingX && armPosition >= 2630) {
				Arm.brake();
				presettingX = false;
			}

			if (presettingA && armPosition >= 1600) {
				Arm.brake();
				presettingA = false;
			}
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

	// input

		if (Master.get_digital(DIGITAL_L2) == true) {
			if (InputPiston.get_value() == false) {
				InputPiston.set_value(true);
			}
			else {
				InputPiston.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_L2) == false);
		}


	// color sensor

		//                        < 020
		if ((colorSense.get_hue() > 200) && (toggleColorSensor == true) && (autonnumber < 0)) {
			Eject.set_value(true);
			colorDelay = 1;
		} if ((colorSense.get_hue() < 20) && (toggleColorSensor == true) && (autonnumber > 0)) {
			Eject.set_value(true);
			colorDelay = 1;
		} else if (colorDelay >= 500) {
			Eject.set_value(false);
			colorDelay = 0;
		}
		if (colorDelay != 0) {
			colorDelay += 20;
		}

		if (Master.get_digital(DIGITAL_R2) && toggleColorSensor == false) {
			toggleColorSensor = true;
		} else if (Master.get_digital(DIGITAL_R2) && toggleColorSensor == true) {
			toggleColorSensor = false;
			colorDelay = 0;
			Eject.set_value(false);
		} waitUntil(!Master.get_digital(DIGITAL_R2));

		Master.print(0, 0, "toggle = %d", toggleColorSensor);




	// grab arm

		if (intakePTOvalue == false && Master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			if (GrabPiston.get_value() == false) {
				GrabPiston.set_value(true);
			}
			else {
				GrabPiston.set_value(false);
			}
			waitUntil(Master.get_digital(DIGITAL_X) == false);
		}
*/
	pros::delay(20);

	}



}
