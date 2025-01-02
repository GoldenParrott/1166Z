#include "init.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

	pros::lcd::initialize();
	pros::lcd::print(1, "I SUCK");
	Master.print(0, 0, "I REFUSE TO WORK");

	Intake.move_relative(1000, 600);
	
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
void competition_initialize() {

	// autoSelector_task_ptr = new pros::Task(drawAutonSelector);
	autonnumber = -5;
	globalAuton = false;


	while (true) {
		if (globalAuton == true) {
			switch (autonnumber) {
				case 1: //Blue Mogo
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {50, -36}, 251);
					break;
				case 2:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {54.5, 13.125}, 208);
					break;
				case -1:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-49.25, -60.325}, 69);
					break;
				case -2: //Red Ring
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-55, 12}, 148);
					break;
				case 3:
				case -3: //Test (?)
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-48, -48}, 225);
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
				case 2:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {0, 0}, 0);
					break;
				case -2:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-54, 12.5}, 232);
					break;
				case -5:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-60.75, 0}, 90);
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

	Intake.tare_position();

	Arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
	Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Arm.tare_position();

	Kalman1.startFilter();
	Kalman2.startFilter();











	auto RPMtoMPS = [] (double gearset, double gearRatio, double diameter) {
        return (gearset * gearRatio * (3.14 * diameter)) / 60;
    };

    // ROBOT CONFIG
    double gearRatio = 0.75;
    double maxRPM = 600;
    double diameter = 3.25;
    double distBetweenWheels = 10.5;


    double numPoints = 1000;

    double maxSpeed = RPMtoMPS(maxRPM, gearRatio, diameter); // in meters per second


    
    CubicHermiteSpline mySpline = CubicHermiteSpline({0, 0}, {0, -1}, {0.5, -0.5}, {1, -0.5});
    MotionProfile* myProfile = new MotionProfile(mySpline.entirePath(numPoints), maxSpeed);
    VelocityController myController = VelocityController(diameter, distBetweenWheels, gearRatio, maxRPM);
    myController.queueProfile(myProfile);
    myController.startQueuedProfile(false);
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

	Master.print(0, 0, "sup");
	// Front, Middle, Rear
	LeftWheels.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	RightWheels.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	Intake.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	bool clampOn = true;

	// Drving variables
	int drvfb;
	int drvlr;
	int drvtrdz = 10;

	while (true) {
		
	//Drivetrain Control 
		drvfb = Master.get_analog(ANALOG_LEFT_Y);
		drvlr = Master.get_analog(ANALOG_RIGHT_X);

		if ((abs(drvfb) > drvtrdz) || (abs(drvlr) > drvtrdz)) {
      		// ^^ Checks to see if either joystick has moved out of the deadzone
			RightWheels.move((drvfb-(drvlr)));
      		LeftWheels.move((drvfb+(drvlr)));	
    	} else {
			RightWheels.brake();
      		LeftWheels.brake();
    	} 
	
	//Intake Control
	//Arm up Y
	//Arm down B
		if(Master.get_digital(DIGITAL_RIGHT))
		{
			Intake.move(127);
		}
		else if(Master.get_digital(DIGITAL_DOWN))
		{
			Intake.move(-127);
		}
		else if(Master.get_digital(DIGITAL_R2))
		{
			Preroller.move(127);
		}
		else
		{
			Intake.brake();
		}

	//Arm Control
		if(Master.get_digital(DIGITAL_Y))
		{
			Arm.move(127);
		}
		else if(Master.get_digital(DIGITAL_B))
		{
			Arm.move(-127);
		}
		else
		{
			Arm.brake();
		}			

		//Mogo
		if(Master.get_digital_new_press(DIGITAL_R1)){

			// ↓↓ If the manipulator is open, activate this code
			if (Clamp.get_value() == false) {

				// ↓↓ Closes the manipulator to grab an object
				Clamp.set_value(true);

			// ↓↓ If the manipulator is closed, activate this code
			} else if (Clamp.get_value() == true){

				// ↓↓ Opens the manipulator to grab an object
				Clamp.set_value(false);
			}
		}
		
		
		pros::delay(20);
	}
}