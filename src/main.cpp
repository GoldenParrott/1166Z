#include "main.h"
#include "init.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

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
	while (true) {

	//Drivetrain
    	drvtrFB = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    	drvtrLR = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if ((abs(drvtrFB) > drvtrDZ) || (abs(drvtrLR) > drvtrDZ)) {
      		// ^^ Checks to see if either joystick has moved out of the deadzone
			if (intakePTOvalue == true){

				RightWheels.move((drvtrFB-(drvtrLR)));
      			LeftWheels.move((drvtrFB+(drvtrLR)));
			
			} else if (intakePTOvalue == false){

				AllRightWheels.move((drvtrFB-(drvtrLR)));
      			AllLeftWheels.move((drvtrFB+(drvtrLR)));
			
			}
      			
    	} else {
			
			if (intakePTOvalue == true){

				RightWheels.brake();
      			LeftWheels.brake();
			
			} else if (intakePTOvalue == false){

				AllRightWheels.brake();
      			AllLeftWheels.brake();
			
			}
      			
    	}  

	//Intake Conveyor (Transport)
		if (Master.get_digital(DIGITAL_RIGHT)){
			Intake.move(-128);
		} else if(Master.get_digital(DIGITAL_LEFT)){
			Intake.move(128);
		} else {
			Intake.brake();
		}

	// Intake Arm
		if (intakePTOvalue == true) {
			if (Master.get_digital(DIGITAL_UP)) {
				IntakePTO.move(-128);
			}else if (Master.get_digital(DIGITAL_DOWN)) {
				IntakePTO.move(128);
			}
		}

	// Intake PTO
		if (Master.get_digital(DIGITAL_X)) {
			if (!intakePTOvalue) {
				IntakePTOPiston.set_value(true);
				intakePTOvalue = true;
			} else {
				IntakePTOPiston.set_value(false);
				intakePTOvalue = false;
			}
		}

	pros::delay(20);

	}
}
