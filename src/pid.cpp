#include "init.h"

void PIDMover(
		int setPoint, // how far you want to move in inches

		std::function<void(void)> custom, // a lambda function that will execute during the PID (optional)
		int executeAt // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{

// checks if the PTO is on or not to ensure that the correct sets of motors are used later
	bool PTOon;
	if (IntakePTOPiston.get_value() == true) {PTOon = true;}
	else {PTOon = false;}


	AllAllWheels.set_brake_modes(MOTOR_BRAKE_HOLD);

// PID Calculation Variables
	// General Variables
	int error;
	int power;
	int tolerance = 2;
	int cyclesAtGoal;
	bool customCompleted = false;
	bool actionCompleted = false;

	// Proportional Variables
	int proportionalOut;

	// Integral Variables
	int integral = 0;
	int integralLimiter = 512; // customizable
	int integralOut;

	// Derivative Variables
    int derivative;
    int derivativeOut;
	int prevError;

	// Constants (need to be tuned individually for every robot)
	double kP = 1.28; // customizable
	double kI = 0.4; // customizable
	double kD = 0.1; // customizable

	
	




// Odometry Measurement Setup
	bool isPositive = setPoint > 0; // Checks if the movement is positive or negatives
	setPoint = setPoint * 2.54; // converts from inches to cm, as the function call uses inches for ease of measurement
	double gearRatio = 0.75; // the gear ratio of the robot (gear axle / motor axle)

	executeAt = executeAt * 2.54;

	double wheelCircumference = 3.14 * 3.25; // 3.25 is the wheel diameter in inches
	double wheelRevolution = wheelCircumference * 2.54; // wheel circumference in cm
						// this is equivalent to how far the robot moves in one 360-degree rotation of its wheels
	long double singleDegree = wheelRevolution / 360; // the distance that the robot moves in one degree of rotation of its wheels



// Odometry Pre-Measurement
	// resets the rotation of all motors before the movement so the movement can be calculated from zero to the destination
	BackRight.tare_position();
	BackLeft.tare_position();
	FrontRight.tare_position();
	FrontLeft.tare_position();

	// used to measure the rotational sensor values of all the motors (this comes in degrees)
	double br = BackRight.get_position();
	double bl = BackLeft.get_position();
	double fr = FrontRight.get_position();
	double fl = FrontLeft.get_position();

	double currentMotorReading = ((br + bl + fr + fl) / 4); // measures the average rotation of all motors to determine the movement of the entire robot
	double currentWheelReading = currentMotorReading * gearRatio; // measures the current reading (in degrees) of the wheel by multiplying it by the gear ratio

	// measures the current distance moved by the robot by multiplying the number of degrees that it has moved 
	// by the number of centimeters moved in a single degree of movement
	double currentDistanceMovedByWheel = currentWheelReading * singleDegree; 

	// these initialize variables that are used to measure values from previous cycles
	error = (int) (setPoint - currentDistanceMovedByWheel);
	prevError = error;

	 

	while (actionCompleted != true) {
// PID Looping Calculations
		
	// P: Proportional -- slows down as we reach our target for more accuracy

		// error = goal reading - current reading
		error = int (setPoint - currentDistanceMovedByWheel);
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = int (integral + error);
		// prevents the integral variable from causing the robot to overshoot
		if ((isPositive && (error <= 0)) || (!isPositive && (error >= 0))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		if (((isPositive) && (error >= 100)) || ((!isPositive) && (error <= -100))) {
			integral = 0;
			}
		if (((isPositive) && (integral > 100)) || ((!isPositive) && (integral < -100))) {
			integral = isPositive
				? 100
				: -100;
			}
		// kI (integral constant) brings integral down to a reasonable/useful output number
		integralOut = integral * kI;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
        derivative = int (error - prevError);
		// sets the previous error to the current error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;



	// Adds the results of each of the calculations together to get the desired power
		power = proportionalOut + integralOut + derivativeOut;

	// moves the wheels at the desired power, ending the cycle
	if (PTOon) {
		AllWheels.move(power);
	} else if (!PTOon) {
		AllAllWheels.move(power);
	}



	// Custom lambda function that will execute if given and the robot has reached the point given by executeAt
		
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (custom != 0 && currentDistanceMovedByWheel >= executeAt && !customCompleted) {
			// runs the function
			custom();
			// prevents the function from running again
			customCompleted = true;
		}


// PID Looping Odometry Measurement

		// fifteen millisecond delay between cycles
		pros::delay(15);

		// finds the degrees of measurement of the motors
		br = BackRight.get_position();
		bl = BackLeft.get_position();
		fr = FrontRight.get_position();
		fl = FrontLeft.get_position();

		// reassigns the "distance moved" variables for the next cycle after the delay
		currentMotorReading = ((br + bl + fr + fl) / 4); // degrees
		currentWheelReading = currentMotorReading * gearRatio; // degrees = degrees * gear ratio multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		// checks to see if the robot has completed the movement by checking several conditions, and ends the movement if needed
		if (((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) {
			// if (cyclesAtGoal >= 20) {
				actionCompleted = true;
				if (PTOon) {AllWheels.brake();}
				else if (!PTOon) {AllAllWheels.brake();}
		}
	}
}

void PIDTurner(
		int setPoint, // the goal inertial heading in degrees
		int direction, // 1 for left and 2 for right

		std::function<void(void)> custom, // a lambda function that will execute during the PID (optional)
		int executeAt // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{

// checks if the PTO is on or not to ensure that the correct sets of motors are used later
	bool PTOon;
	if (IntakePTOPiston.get_value() == true) {PTOon = true;}
	else {PTOon = false;}

	AllAllWheels.set_brake_modes(MOTOR_BRAKE_HOLD);

// PID CALCULATION VARIABLES
// General Variables
	int error;
	int power;
	int tolerance = 1;
	bool customCompleted = true;
	bool actionCompleted = false;

// Proportional Variables
	int proportionalOut;

// Integral Variables
	int integral;
	int integralLimiter;
	int integralOut;

// Derivative Variables
    int derivative;
    int derivativeOut;
	int prevError = error;

// Constants -- tuning depends on whether the robot is moving or turning
	double kP;
	double kI;
	double kD;


// Checks if the movement is positive or negative
	bool isPositive = setPoint > 0;

// PID LOOPING VARIABLES
	int negativePower;

	int inertialReadingInit = Inertial.get_heading();
	int distanceToMove;

	if (direction == 1) {
		// standard left turn is negative, so the calculation makes it positive if it is a normal turn
		// ex: current = 90, goal = 45 -> -45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn left
		distanceToMove = inertialReadingInit - setPoint;
	}
	else if (direction == 2) {
		// standard right turn is positive, so the calculation keeps it positive if it is a normal turn
		// ex: current = 45, goal = 90 -> 45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn right
		distanceToMove = setPoint - inertialReadingInit;
	}

	// if the error is positive, then the calculation is fine and is left
	if (distanceToMove >= 0) {
		// do nothing
	}
	// otherwise, the turn takes the "long way" around the circle, and the calculation has provided the
	// value of the negative short way - adding 360 to this value gives the long way around the circle,
	// which is what is needed
	// ex: current = 90, goal = 45, direction = right -> calculated -45 degree turn -> + 360 -> 315 (length of long way)
	// 45 - 90 = -45 (short way, negative) + 360 = 315 (long way, positive)
	else {
		distanceToMove += 360;
	}
	// the calculation has now yielded a positive value that is the error needed of our turn in the proper
	// direction, making it similar to how a forward/backward movement is coded

	// finally, the code sets a new value that will be set to the distance moved to zero to finalize this similarity
	// distanceToMove is analogous to setPoint on PIDMover, and changeInReading is analogous to currentDistanceMovedByWheel
	int changeInReading = 0;

	prevError = (int) (distanceToMove - changeInReading);



	distanceToMove -= 2;

	if (distanceToMove <= 90) {
		kP = 1.35;
		kI = 0.19;
		kD = 0.1;
	} else {
		kP = 0.9;
		kI = 0.13;
		kD = 0.2;
	}

	 

	while (!actionCompleted) {
	// PID CALCULATION CODE
		
	// P: Proportional -- slows down as we reach our target for more accuracy
	
		// error = goal reading - current reading
		error = distanceToMove - changeInReading;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = int (integral + error);
		// prevents the integral variable from causing the robot to overshoot
		if ((isPositive && (error <= 0)) || (!isPositive && (error >= 0))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		if (((isPositive) && (error >= 135)) || ((!isPositive) && (error <= -135))) {
			integral = 0;
			}
		if (((isPositive) && (integral > 100)) || ((!isPositive) && (integral < -100))) {
			integral = isPositive
				? 100
				: -100;
			}
		// kI (integral constant) brings integral down to a reasonable/useful output number
		integralOut = integral * kI;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
		// the error from the previous cycle should be taken as a parameter
        derivative = int (error - prevError);
		// sets the previous error to the previous error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;

		power = proportionalOut + integralOut + derivativeOut;




		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (custom != 0 && changeInReading >= executeAt && !customCompleted) {
			// runs the function
			custom();
			// prevents the function from running again
			customCompleted = true;
		}



	// PID LOOPING CODE

		negativePower = power * -1;

		// the power will never be negative and invert the turns because distanceToMove is always positive
		if (PTOon) {
			if (direction == 1) {
				LeftWheels.move(negativePower);
				RightWheels.move(power);
			}
			else if (direction == 2) {
				LeftWheels.move(power);
				RightWheels.move(negativePower);
			}
		} else if (!PTOon) {
			if (direction == 1) {
				AllLeftWheels.move(negativePower);
				AllRightWheels.move(power);
			}
			else if (direction == 2) {
				AllLeftWheels.move(power);
				AllRightWheels.move(negativePower);
			}
		}

		pros::delay(15);

		// the change in reading is set to the absolute value of the change in reading due to everything being positive
		int changeInDistance = direction == 1 
			? inertialReadingInit - Inertial.get_heading() 
			: Inertial.get_heading() - inertialReadingInit;
		changeInReading = changeInDistance < 0
		    ? changeInDistance + 360
			: changeInDistance;
		

		if (((changeInReading <= (distanceToMove + tolerance)) && (changeInReading >= (distanceToMove - tolerance)))) {
				actionCompleted = true;
				if (PTOon) {AllWheels.brake();}
				else if (!PTOon) {AllAllWheels.brake();}
		}
	}
}

void PIDArc(
	int chordLength, // the distance between the robot's starting position and its destination position
	int maxDist, // the maximum distance of the straight line from your current position and the setPoint to the arc (should be measured at half-point)
	int direction, // 1 for left, 2 for right

	std::function<void(void)> custom, // a lambda function that will execute during the PID (optional)
	int executeAt // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
	)
{
// Checks if the movement is positive or negative
	bool isPositive = chordLength > 0;


// checks if the PTO is on or not to ensure that the correct sets of motors are used later
	bool PTOon;
	if (IntakePTOPiston.get_value() == true) {PTOon = true;}
	else {PTOon = false;}


	AllAllWheels.set_brake_modes(MOTOR_BRAKE_HOLD);


// PID CALCULATION VARIABLES
// General Variables
	int error;
	int power;
	int tolerance = 1;
	bool customCompleted = false;
	bool actionCompleted = false;

// Proportional Variables
	int proportionalOut;

// Integral Variables
	int integral = 0;
	int integralLimiter = 512; // customizable
	int integralOut;

// Derivative Variables
    int derivative;
    int derivativeOut;
	int prevError;
	

// Constants -- tuning depends on whether the robot is moving or turning
	double kP = 3.2; // customizable
	double kI = 0.3; // customizable
	double kD = 0.1; // customizable


// PID LOOPING VARIABLES
	chordLength = chordLength * 2.54; // converts from inches to cm
	maxDist = maxDist * 2.54;


// Odometry
	double wheelCircumference = 3.14 * 3.25; // 3.25 is the wheel diameter in inches
	double gearRatio = 3 / 4;
	double wheelRevolution = wheelCircumference * 2.54; // in cm
	long double singleDegree = wheelRevolution / 360;

	BackRight.tare_position();
	BackLeft.tare_position();
	FrontRight.tare_position();
	FrontLeft.tare_position();

	double bo;
	double fo;

	double currentMotorReading = ((bo + fo) / 2);
	double currentWheelReading = currentMotorReading * gearRatio;

	double currentDistanceMovedByWheel = 0;

	error = (int) (chordLength - currentDistanceMovedByWheel);
	prevError = error;

// Arc Measurement
	if (!isPositive) {chordLength = -chordLength;}
	double halfSetPoint = chordLength / 2; // divides the chord/setPoint into two halves, one on each side of the bisector maxDist
	double diameterMinusMaxDist = (halfSetPoint * halfSetPoint) / maxDist; // uses the Intersecting Chords Theorem to find the diameter of the circle (excluding maxDist)
	double diameter = diameterMinusMaxDist + maxDist; // adds maxDist to the previous variable to find the diameter of the circle
	double radius = diameter / 2; // basic circle math - the radius is half of the diameter
	double centralAngleOfArc = std::acos(((radius * radius) + (radius * radius) - (chordLength * chordLength)) / (2 * radius * radius)); // Law of Cosines to find central angle - a and b are the radius, and c is the chord (returns radians)
	double arcLength = centralAngleOfArc * radius; // arc length formula (angle in radians * radius)
	double setPoint = arcLength; // sets the setPoint to the length of the arc instead of the length of the chord, as it will actually be moving that far


// Arc Odometry
	double oRadius = (((halfSetPoint * halfSetPoint) / maxDist) + maxDist) / 2;
	double distBetweenWheels = 13 * 2.54;
	double mult = ((oRadius - distBetweenWheels) / oRadius);

// Arc Turning - turns the robot so that it starts the movement perpendicular to the center of the circle so it can move along the arc accurately
	double angleOfArcFromStartPosRAD = std::acos(((radius * radius) + (chordLength * chordLength) - (radius * radius)) / (2 * radius * chordLength)); // Law of Cosines to find starting angle of arc - a and c are the radius, and b is the chord (returns radians)
	int angleOfArcFromStartPosDEG = (int) (angleOfArcFromStartPosRAD * (180 / 3.14)); // converts the radians to degrees
	int angleToTurn = 90 - angleOfArcFromStartPosDEG; // as we are already facing toward the destination, this sets our value to turn as the angle between the chord and a line perpendicular to the radius of the circle
	int directionForTurn = direction == 1
		? 2
		: 1;
	int newHeading = directionForTurn == 1
		? Inertial.get_heading() - angleToTurn // negative left turn
		: Inertial.get_heading() + angleToTurn; // positive right turn

	// if (newHeading < 0) {newHeading = newHeading + 360;} // makes the new heading the actual new heading if it is a negative

	PIDTurner(newHeading, directionForTurn);

	if (!isPositive) {setPoint = -setPoint;}


	while (actionCompleted != true) {
	// PID CALCULATION CODE
		
	// P: Proportional -- slows down as we reach our target for more accuracy

		// error = goal reading - current reading
		error = int (setPoint - currentDistanceMovedByWheel);
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = int (integral + error);
		// prevents the integral variable from causing the robot to overshoot
		if ((isPositive && (error == 0)) || (!isPositive && (error == 0))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		if (((isPositive) && (error >= 100)) || ((!isPositive) && (error <= -100))) {
			integral = 0;
			}
		if (((isPositive) && (integral > 100)) || ((!isPositive) && (integral < -100))) {
			integral = isPositive
				? 100
				: -100;
			}
		// kI (integral constant) brings integral down to a reasonable/useful output number
		integralOut = integral * kI;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
        derivative = int (error - prevError);
		// sets the previous error to the previous error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;




		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (custom != 0 && currentDistanceMovedByWheel >= executeAt && !customCompleted) {
			// runs the function
			custom();
			// prevents the function from running again
			customCompleted = true;

		}





		power = proportionalOut + integralOut + derivativeOut;

	// caps motor power at 128 if it goes beyond it to ensure that the multiplier makes one side spin slower
		if (power > 128) {
			power = 128;
		}
		if (power < -128) {
			power = -128;
		}

		// causes the wheels to move in the proper direction, with the outer wheels being normal and the inner wheels being multiplied by mult
		if (PTOon) {
			if (direction == 1) {
				RightWheels.move(power);
				LeftWheels.move(power * mult);
			} else if (direction == 2) {
				RightWheels.move(power * mult);
				LeftWheels.move(power);
			}
		} else if (!PTOon) {
			if (direction == 1) {
				AllRightWheels.move(power);
				AllLeftWheels.move(power * mult);
			} else if (direction == 2) {
				AllRightWheels.move(power * mult);
				AllLeftWheels.move(power);
			}
		}


		pros::delay(15);

		if (direction == 1) {
			bo = BackRight.get_position();
			fo = FrontRight.get_position();
		} else {
			bo = BackLeft.get_position();
			fo = FrontLeft.get_position();
		}

		currentMotorReading = ((bo + fo) / 2); // degrees
		currentWheelReading = currentMotorReading / gearRatio; // degrees = degrees * multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		if (((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) {
				actionCompleted = true;
				if (PTOon) {AllWheels.brake();}
				else if (!PTOon) {AllAllWheels.brake();}
		}
	}
}
