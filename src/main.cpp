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

	AllAllWheels.move_relative(1000,50);
	pros::delay(2000);
	AllAllWheels.move_relative(-1000,50);



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

	/*
	Key:
		Right & Left : Intake

	
	
	*/
	
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

	// Intake Conveyor (Transport)
		if (Master.get_digital(DIGITAL_RIGHT)){
			Intake.move((-128));
		} else if(Master.get_digital(DIGITAL_LEFT)){
			Intake.move((128));
		} else {
			Intake.brake();
		}

	// Intake Arm
//hif
		// The button 'LowerLimit' is used to set a reference point of the bottom of the arm's 
		// rotation point. The button gets pressed when our arm is all the way down.

		// ↓↓ If the button is pressed, run this code
		if (LowerLimit.get_value() == true){

			// ↓↓ Sets rotation sensor is the arm to zero 
			UpRight.tare_position();

			// ↓↓ Sets the 
			armmax = 1850.0;
			armCalibrated = true;
		} else if(LowerLimit.get_value() == false){

		}


		armPosition = abs(UpRight.get_position());

		if (intakePTOvalue == true) {
			if ((Master.get_digital(DIGITAL_DOWN))/*&&(LowerLimit.get_value() == false)*/) {
				IntakePTO.move(128);
			} else if ((Master.get_digital(DIGITAL_UP))/*&&(armPosition<(armmax))*/) {
				IntakePTO.move(-128);
				
			} else {
				IntakePTO.brake();
			}

			/*
			// Need to recalibrate values, is preventing manual arm lifting 
			if ((Master.get_digital(DIGITAL_X) == true)){
				//Up
				armGoal = 1900;
				if (armGoal < armPosition) {
					IntakePTO.move_relative((armPosition-armGoal),128);
				} else if (armPosition < armGoal) {
					IntakePTO.move_relative(-(armGoal-armPosition),128);
				}
			}else if((Master.get_digital(DIGITAL_B) == true)){
				//Down
				armGoal = 0;
				if (armGoal < armPosition) {
					IntakePTO.move_relative((armPosition-armGoal),128);
				} else if (armPosition < armGoal) {
					IntakePTO.move_relative(-(armGoal-armPosition),128);
				}
			}else if (Master.get_digital(DIGITAL_A)){
				//Midpoint
				armGoal = 800;
				if (armGoal < armPosition) {
					IntakePTO.move_relative((armPosition-armGoal),128);
				} else if (armPosition < armGoal) {
					IntakePTO.move_relative(-(armGoal-armPosition),128);
				}
				
			}
			*/
		}

		if (Master.get_digital(DIGITAL_A)){
			IntakePTO.brake();
		}


	// Intake PTO

		// ↓↓ Pressing the Y Button toggles between modes
		if (Master.get_digital(DIGITAL_Y)) {

			// intakePTOvalue is a variable that gets changed
			// between true and false every time the Y Button 
			// is pressed to switch modes

			// ↓↓ If PTO piston is deactivated, activate this code
			if (intakePTOvalue == false) {

				// Sets the PTO piston to change the connected 
				// gear train ↓↓
				IntakePTOPiston.set_value(true);

				// Sets variable to allow the other portion of
				// the code to run ↓↓
				intakePTOvalue = true;

				// Sets the motors on the PTO to HOLD their position
				// while in control of the arm, and the wheels on the
				// drivetrain to COAST to a stop ↓↓
				IntakePTO.set_brake_modes(MOTOR_BRAKE_HOLD);
				AllWheels.set_brake_modes(MOTOR_BRAKE_COAST);

			// ↓↓ If PTO piston is activated, activate this code
			} else if (intakePTOvalue == true) {

				// Sets the PTO piston to change the connected 
				// gear train ↓↓
				IntakePTOPiston.set_value(false);

				// Sets variable to allow the other portion of
				// the code to run ↓↓
				intakePTOvalue = false;

				// Sets all motors now attached to the drivetrain
				// to COAST to a stop ↓↓
				AllAllWheels.set_brake_modes(MOTOR_BRAKE_COAST);
			}

			// Sets a condition to exit the code that is the 
			// opposite of the condition to enter, preventing
			// us from looping through the code repeatedly ↓↓
			waitUntil(Master.get_digital(DIGITAL_Y) == false);
		}

		if (!intakePTOvalue) {
			//Master.print(0, 0, "PTO in arm mode");
		}

	//Mobile Goal Manipulator

		// ↓↓ Pressing the R1 Button toggles between modes
		if(Master.get_digital(DIGITAL_R1)){

			// mobileGoalManipulatorValue is a variable that gets changed
			// between true and false every time the R1 Button 
			// is pressed to switch modes

			// ↓↓ If the manipulator is open, activate this code
			if (mobileGoalManipulatorValue == false) {

				// ↓↓ Closes the manipulator to grab an object
				MobileGoalManipulator.set_value(true);

				// Sets variable to allow the other portion of
				// the code to run ↓↓
				mobileGoalManipulatorValue = true;

			// ↓↓ If the manipulator is closed, activate this code
			} else if (mobileGoalManipulatorValue == true){

				// ↓↓ Opens the manipulator to grab an object
				MobileGoalManipulator.set_value(false);

				// Sets variable to allow the other portion of
				// the code to run ↓↓
				mobileGoalManipulatorValue = false;
			}

			// Sets a condition to exit the code that is the 
			// opposite of the condition to enter, preventing
			// us from looping through the code repeatedly ↓↓
			waitUntil(Master.get_digital(DIGITAL_R1) == false);
		}


	pros::delay(20);

	}
}
