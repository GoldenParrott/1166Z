#include "main.h"

/*
 * Runs the arm up or down to a defined location. There are 15 degrees of buffer around
 * 
 * goal is the degree of rotation of the motors from 0-1900
*/
/*
void armraiser(double goal){
	// Makes a new variable for the arm position. The value gotten is a negative so we 
	// change that to a posive value before moving on

	double current = abs(UpRight.get_position());
	
	if (((goal - 15) < current) && (current < (goal + 15))){
		// Used if the arm is within the threashold of the desired position
		IntakePTO.brake();
	} else if (current >= (goal + 15)){
		// Used if the arm is above the desired position
		if ((current - goal) >= 200){
			// Used if the arm is far away from the desired position
			// Runs motors at max speed
			IntakePTO.move(-128);
		} else if ((current - goal) < 200){
			// Used if the arm is close to the desired position
			// Runs motors at ~1/3 speed for more accurate movement
			IntakePTO.move(-43);
		}
	} else if (current <= (goal - 15)) {
		Master.print(0,0,"c=%f, g=%f",current, goal);
		// Used if the arm is below the desired position
		if ((goal - current) >= 200){
			// Used if the arm is far away from the desired position
			// Runs motors at max speed
			IntakePTO.move(128);
		} else if ((goal - current) < 200){
			// Used if the arm is close to the desired position
			// Runs motors at ~1/3 speed for more accurate movement
			IntakePTO.move(43);
		}
	} else {
		Master.print(0,0,"Listen man, I don't even know");
	}
}
*/

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
		pros::lcd::read_buttons();
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
void autonomous() {

	
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

	IntakePTOPiston.set_value(false);
	intakePTOvalue = false;

	
	
	AllAllWheels.move_velocity(1000);
	AllAllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);

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
		
		if (LowerLimit.get_value() == true){
			UpRight.tare_position(); //set to zero
			armmax = 1850.0;
			pros::lcd::set_text(1, "Limit Hit");
			armCalibrated = true;
		}
		if (LowerLimit.get_value() == false){
			pros::lcd::set_text(1, "Limit Released");
		}

		armpos = abs(UpRight.get_position());
		//Master.print(0, 0, "%f",UpRight.get_position());
		//Master.clear();

		if (intakePTOvalue == true) {
			if ((Master.get_digital(DIGITAL_DOWN))&&(LowerLimit.get_value() == false)) {
				IntakePTO.move(128);
			}else if ((Master.get_digital(DIGITAL_UP))&&(armpos<(armmax))) {
				IntakePTO.move(-128);
			}
		}

		if ((Master.get_digital(DIGITAL_X) == true)){
			//Up
			armGoal = 1900;
			if (armGoal < armpos) {
				IntakePTO.move_relative((armpos-armGoal),128);
			} else if (armpos < armGoal) {
				IntakePTO.move_relative(-(armGoal-armpos),128);
			}
		}else if((Master.get_digital(DIGITAL_B) == true)){
			//Down
			armGoal = 0;
			if (armGoal < armpos) {
				IntakePTO.move_relative((armpos-armGoal),128);
			} else if (armpos < armGoal) {
				IntakePTO.move_relative(-(armGoal-armpos),128);
			}
		}else if (Master.get_digital(DIGITAL_A)){
			//Midpoint
			armGoal = 800;
			if (armGoal < armpos) {
				IntakePTO.move_relative((armpos-armGoal),128);
			} else if (armpos < armGoal) {
				IntakePTO.move_relative(-(armGoal-armpos),128);
			}
			
		}

		if (Master.get_digital(DIGITAL_A)){
			IntakePTO.brake();
		}

	// Intake PTO
		if (Master.get_digital(DIGITAL_Y)) {
			if (!intakePTOvalue) {
				IntakePTOPiston.set_value(true);
				intakePTOvalue = true;
				IntakePTO.set_brake_modes(MOTOR_BRAKE_HOLD);
				AllWheels.set_brake_modes(MOTOR_BRAKE_COAST);
			} else {
				IntakePTOPiston.set_value(false);
				intakePTOvalue = false;
				AllAllWheels.set_brake_modes(MOTOR_BRAKE_COAST);
			}

			waitUntil(Master.get_digital(DIGITAL_Y) == false);
		}
		if (!intakePTOvalue) {
			//Master.print(0, 0, "PTO in arm mode");
		}

	//Mgm
		if(Master.get_digital(DIGITAL_R1)){
			if (!mgmValue) {
				Mgm.set_value(true);
				mgmValue = true;
			} else {
				Mgm.set_value(false);
				mgmValue = false;
			}

			waitUntil(Master.get_digital(DIGITAL_R1) == false);
		}


	pros::delay(20);

	}
}
