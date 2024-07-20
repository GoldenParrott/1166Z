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



	autonnumber = 1;
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
	MobileGoalManipulator.set_value(false);
	InputPiston.set_value(false);
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

	// MoGo Auton
	// autonomous setup
	AllAllWheels.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Transport.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Transport.tare_position();
	UpLeft.set_encoder_units(MOTOR_ENCODER_DEGREES);
	UpLeft.tare_position();
	pros::Task colorSensorOn_task(colorSensorOn, "Color Eject On");


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
	//Master.print(0, 0, "Done");

	pros::delay(1000);
	AllAllWheels.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	colorSensorOn_task.remove();
	GrabPiston.set_value(false);
	Eject.set_value(false);

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
	Eject.set_value(false);
	GrabPiston.set_value(false);

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

	// Intake Conveyor (Transport) and Input
		if (Master.get_digital(DIGITAL_RIGHT)){
			InputMotor.move(-128);
			Transport.move(-128);
		} else if(Master.get_digital(DIGITAL_LEFT)){
			InputMotor.move(128);
			Transport.move(128);
		}
	// Input Only

		if (intakePTOvalue == false)
		{
			if (Master.get_digital(DIGITAL_L1)) 
			{
				InputMotor.move(-128);
			}
			else if(Master.get_digital(DIGITAL_DOWN))
			{
				InputMotor.move(128);
			} 
			else if ((Master.get_digital(DIGITAL_RIGHT) == false) && (Master.get_digital(DIGITAL_LEFT) == false) && Master.get_digital(DIGITAL_DOWN) == false && Master.get_digital(DIGITAL_L1) == false) {
				InputMotor.brake();
			}
			if (intakePTOvalue == false && Master.get_digital(DIGITAL_DOWN) == true){
				InputMotor.move(128);
			}
		}
		else 
		{
			if (Master.get_digital(DIGITAL_L1)) {
				InputMotor.move(-128);
			} else if ((Master.get_digital(DIGITAL_RIGHT) == false) && (Master.get_digital(DIGITAL_LEFT) == false)) {
				InputMotor.brake();
			}
		}
		
	// Transport Only
		if (Master.get_digital(DIGITAL_B)) {
			Transport.move(128);
		} else if ((Master.get_digital(DIGITAL_RIGHT) == false) && (Master.get_digital(DIGITAL_LEFT) == false)) {
			Transport.brake();
		}

	// Intake Arm

		// When the intake PTO first switches on, this code resets the zero position of the arm to be the bottom (where it is currently at)
		// as a reference point

		// ↓↓ If the PTO is switched on, run this code
		if (intakePTOvalue == true && !armCalibrated) {
			// ↓↓ Sets the rotational sensor in the arm motor to zero as a reference point
			UpLeft.tare_position();
			// changes the armCalibrated value to ensure that this code is not run again until the next time it is switched on
			armCalibrated = true;
		}

		// switches the armCalibrated value back to false for the next time when the PTO is first switched off
		if (intakePTOvalue == false && armCalibrated) {
			armCalibrated = false;
		}


		armPosition = abs(UpLeft.get_position());

		if (intakePTOvalue == true) {
			if (!presettingX && !presettingA) {
				if ((Master.get_digital(DIGITAL_DOWN))/*&&(LowerLimit.get_value() == false)*/) {
					IntakePTO.move(128);
				} else if ((Master.get_digital(DIGITAL_UP))/*&&(armPosition<(armmax))*/) {
					IntakePTO.move(-128);
				} else {
					IntakePTO.brake();
				}
			}


		// Arm Presets


		// Neutral Stake Preset
			if (Master.get_digital(DIGITAL_X) && !presettingA) {
				IntakePTO.move(-128);
				presettingX = true;
			} 
		// Alliance Stake Preset
			else if (Master.get_digital(DIGITAL_A) && !presettingX) {
				IntakePTO.move(-128);
				presettingA = true;
			}

		// These stop the preset movements in their own separate check to prevent the code from blocking other code
			if (presettingX && armPosition >= 2630) {
				IntakePTO.brake();
				presettingX = false;
			}

			if (presettingA && armPosition >= 1700) {
				IntakePTO.brake();
				presettingA = false;
			}
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
		if ((colorSense.get_hue() > 150) && (toggleColorSensor == true)) {
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

	pros::delay(20);

	}
}
