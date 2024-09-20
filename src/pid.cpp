#include "init.h"

void PIDMover(
		Coordinate goalPosition, // goal coordinate position
		bool reverse, // defaults to false- explicitly set to true to reverse the robot

		std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
		std::vector<int> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{


	AllWheels.set_brake_modes(MOTOR_BRAKE_HOLD);

// PID Calculation Variables
	// General Variables
	double tolerance = 1;
	std::vector<bool> customsCompleted;
	bool actionCompleted = false;

	// Constants (need to be tuned individually for every robot)
	ConstantContainer moverConstants;
	moverConstants.kP = 5; // customizable
	moverConstants.kI = 0.0; // customizable
	moverConstants.kD = 0.0; // customizable

	



// sets the set point to the difference between the current point and the goal point
	Coordinate originalPosition = universalCurrentLocation;
	double setPoint = calculateDistance(originalPosition, goalPosition);
	double remainingDistance = setPoint;

// finds the part of the coordinate plane in which the robot has passed its destination
	Line negativeSide = calculatePerpendicular(originalPosition, goalPosition);

// Odometry Measurement Setup
	bool isPositive = setPoint > 0; // Checks if the movement is positive or negative

	for (int i = 0; i < executeAts.size(); i++) {
		executeAts[i] *= 2.54;
	}



// Odometry Pre-Measurement
	
	// used to measure the rotational sensor values of all the motors (this comes in degrees)
	double currentDistanceMovedByWheel = readOdomPod(Rotational);

	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = setPoint - currentDistanceMovedByWheel;
	cycle.power = 0;
	cycle.prevIntegral = 0;



	while (actionCompleted != true) {

	// gets the power for the current cycle
	cycle = PIDCalc(currentDistanceMovedByWheel, setPoint, isPositive, moverConstants, cycle);
	double power = cycle.power;

	// finds if the robot has passed the perpendicular line's inequality or not
	bool greaterThanNegativeLine = universalCurrentLocation.y >= (negativeSide.slope * universalCurrentLocation.x) + negativeSide.yIntercept;

	// handles the line if it is vertical
	if (std::isnan(negativeSide.slope)) {
		greaterThanNegativeLine = universalCurrentLocation.x > negativeSide.yIntercept;
	} else if (negativeSide.slope == 0) {
		greaterThanNegativeLine = universalCurrentLocation.y > negativeSide.yIntercept;
	}

	// reverses the direction if the robot has passed the inequality
	if ((greaterThanNegativeLine && negativeSide.equality < 0) ||
		(!greaterThanNegativeLine && negativeSide.equality > 0)) {
			power *= -1;
	}
	// reverses the direction if the robot has been commanded to move in reverse
	if (reverse) {
		power *= -1;
	}

	// moves the wheels at the desired power, ending the cycle
	AllWheels.move(power);



	// Custom lambda function that will execute if given and the robot has reached the point given by executeAt
		
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && ((currentDistanceMovedByWheel >= executeAts[i] && isPositive) || (currentDistanceMovedByWheel <= executeAts[i] && !isPositive)) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}


// PID Looping Odometry Measurement

		// fifteen millisecond delay between cycles
		pros::delay(15);

		// calculates the distance moved as the difference between the distance left to move
		// and the total distance to move
		remainingDistance = calculateDistance(universalCurrentLocation, goalPosition);
		currentDistanceMovedByWheel = setPoint - remainingDistance;


		// checks to see if the robot has completed the movement by checking several conditions, and ends the movement if needed
		if (((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) {
				actionCompleted = true;
				AllWheels.brake();
		}
	}

}

void PIDTurner(
		int setPoint, // the goal inertial heading in degrees
		int direction, // 1 for left and 2 for right

		std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
		std::vector<int> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{

	AllWheels.set_brake_modes(MOTOR_BRAKE_HOLD);

// PID CALCULATION VARIABLES
// General Variables
	int error;
	int tolerance = 1;
	std::vector<bool> customsCompleted;
	bool actionCompleted = false;

// Constants -- tuning depends on whether the robot is moving or turning
	ConstantContainer turnerConstants;


// Checks if the movement is positive or negative
	bool isPositive = setPoint > 0;

// PID LOOPING VARIABLES
	int negativePower;

	int inertialReadingInit = getAggregatedHeading(Kalman1, Kalman2);
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





	// constant definitions
	if (distanceToMove <= 90) {
		turnerConstants.kP = 0.75;
		turnerConstants.kI = 0;
		turnerConstants.kD = 0;
	} else {
		turnerConstants.kP = 0.75;
		turnerConstants.kI = 0;
		turnerConstants.kD = 0;
	}

	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = distanceToMove - changeInReading;
	cycle.power = 0;
	cycle.prevIntegral = 0;

	 

	while (!actionCompleted) {
	
	// gets the power for the current cycle
	cycle = PIDCalc(changeInReading, distanceToMove, isPositive, turnerConstants, cycle);
	double power = cycle.power;


	// custom lambda functions
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && ((changeInReading >= executeAts[i] && isPositive) || (changeInReading <= executeAts[i] && !isPositive)) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}



	// PID LOOPING CODE

		negativePower = power * -1;

		// the power will never be negative and invert the turns because distanceToMove is always positive
			if (direction == 1) {
				LeftWheels.move(negativePower);
				RightWheels.move(power);
			}
			else if (direction == 2) {
				LeftWheels.move(power);
				RightWheels.move(negativePower);
			}

		pros::delay(15);

		// the change in reading is set to the absolute value of the change in reading due to everything being positive
		int changeInDistance = direction == 1 
			? inertialReadingInit - getAggregatedHeading(Kalman1, Kalman2)
			: getAggregatedHeading(Kalman1, Kalman2) - inertialReadingInit;
		changeInReading = changeInDistance < 0
		    ? changeInDistance + 360
			: changeInDistance;
		

		if (((changeInReading <= (distanceToMove + tolerance)) && (changeInReading >= (distanceToMove - tolerance)))) {
				actionCompleted = true;
				AllWheels.brake();
		}
	}
}

void PIDArc(
	int chordLength, // the distance between the robot's starting position and its destination position
	int maxDist, // the maximum distance of the straight line from your current position and the setPoint to the arc (should be measured at half-point)
	int direction, // 1 for left, 2 for right

	std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
	std::vector<int> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
	)
{
// Checks if the movement is positive or negative
	bool isPositive = chordLength > 0;


	AllWheels.set_brake_modes(MOTOR_BRAKE_HOLD);


// PID CALCULATION VARIABLES
// General Variables
	int error;
	int power;
	int tolerance = 1;
	std::vector<bool> customsCompleted;
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
	ConstantContainer arcConstants;
	arcConstants.kP = 3.2; // customizable
	arcConstants.kI = 0.3; // customizable
	arcConstants.kD = 0.1; // customizable


// PID LOOPING VARIABLES
	chordLength = chordLength * 2.54; // converts from inches to cm
	maxDist = maxDist * 2.54;
	
	for (int i = 0; i < executeAts.size(); i++) {
		executeAts[i] *= 2.54;
	}


// Odometry
	RotationalTurn.reset();
	double currentDistanceMovedByWheel = readOdomPod(RotationalTurn);

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
		? Inertial1.get_heading() - angleToTurn // negative left turn
		: Inertial1.get_heading() + angleToTurn; // positive right turn

	// if (newHeading < 0) {newHeading = newHeading + 360;} // makes the new heading the actual new heading if it is a negative

	PIDTurner(newHeading, directionForTurn);

	if (!isPositive) {setPoint = -setPoint;}


	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = setPoint - currentDistanceMovedByWheel;
	cycle.power = 0;
	cycle.prevIntegral = 0;


	while (actionCompleted != true) {

	// gets the power for the current cycle
	cycle = PIDCalc(currentDistanceMovedByWheel, setPoint, isPositive, arcConstants, cycle);




	// Custom lambda function that will execute if given and the robot has reached the point given by executeAt
		
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && ((currentDistanceMovedByWheel >= executeAts[i] && isPositive) || (currentDistanceMovedByWheel <= executeAts[i] && !isPositive)) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}

	// caps motor power at 128 if it goes beyond it to ensure that the multiplier makes one side spin slower
		if (power > 128) {
			power = 128;
		}
		if (power < -128) {
			power = -128;
		}

		// causes the wheels to move in the proper direction, with the outer wheels being normal and the inner wheels being multiplied by mult
			if (direction == 1) {
				RightWheels.move(power);
				LeftWheels.move(power * mult);
			} else if (direction == 2) {
				RightWheels.move(power * mult);
				LeftWheels.move(power);
			}


		pros::delay(15);

		currentDistanceMovedByWheel = readOdomPod(RotationalTurn);

		if (((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) {
				actionCompleted = true;
				AllWheels.brake();
		}
	}
}



void PIDArm(
		int setPoint, // how far you want to move in inches

		std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
		std::vector<int> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{


	Arm.set_brake_mode(MOTOR_BRAKE_HOLD);

// PID Calculation Variables
	// General Variables
	int error;
	int power;
	int tolerance = 2;
	std::vector<bool> customsCompleted;
	bool actionCompleted = false;

	// Constants (need to be tuned individually for every robot)
	ConstantContainer moverConstants;
	moverConstants.kP = 1.28; // customizable
	moverConstants.kI = 0.4; // customizable
	moverConstants.kD = 0.1; // customizable

	
	




// Odometry Measurement Setup
	bool isPositive = setPoint > 0; // Checks if the movement is positive or negative
	setPoint = setPoint * 2.54; // converts from inches to cm, as the function call uses inches for ease of measurement

	for (int i = 0; i < executeAts.size(); i++) {
		executeAts[i] *= 2.54;
	}

	double wheelCircumference = 3.14 * 3.25; // 3.25 is the wheel diameter in inches
	double wheelRevolution = wheelCircumference * 2.54; // wheel circumference in cm
						// this is equivalent to how far the robot moves in one 360-degree rotation of its wheels
	long double singleDegree = wheelRevolution / 360; // the distance that the robot moves in one degree of rotation of its wheels



// Odometry Pre-Measurement
	// resets the rotation of all motors before the movement so the movement can be calculated from zero to the destination
	Arm.tare_position();

	// used to measure the rotational sensor values of all the motors (this comes in degrees)
	double armMeasurement = Arm.get_position();

	double currentMotorReading = (armMeasurement / 2); // measures the average rotation of all motors to determine the movement of the entire robot
	double currentWheelReading = currentMotorReading; // measures the current reading (in degrees) of the wheel by multiplying it by the gear ratio

	// measures the current distance moved by the robot by multiplying the number of degrees that it has moved 
	// by the number of centimeters moved in a single degree of movement
	double currentDistanceMovedByWheel = currentWheelReading * singleDegree; 

	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = setPoint - currentDistanceMovedByWheel;
	cycle.power = 0;
	cycle.prevIntegral = 0;

	 

	while (actionCompleted != true) {

	// gets the power for the current cycle
	cycle = PIDCalc(currentDistanceMovedByWheel, setPoint, isPositive, moverConstants, cycle);

	// moves the wheels at the desired power, ending the cycle
	Arm.move(cycle.power);



	// Custom lambda function that will execute if given and the robot has reached the point given by executeAt
		
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && ((currentDistanceMovedByWheel >= executeAts[i] && isPositive) || (currentDistanceMovedByWheel <= executeAts[i] && !isPositive)) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}


// PID Looping Odometry Measurement

		// fifteen millisecond delay between cycles
		pros::delay(15);

		// finds the degrees of measurement of the motors
		armMeasurement = Arm.get_position();

		// reassigns the "distance moved" variables for the next cycle after the delay
		currentMotorReading = (armMeasurement / 2); // degrees
		currentWheelReading = currentMotorReading; // degrees = degrees * gear ratio multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		// checks to see if the robot has completed the movement by checking several conditions, and ends the movement if needed
		if (((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) {
				actionCompleted = true;
				Arm.brake();
		}
	}
}



PIDReturn PIDCalc(
	double distanceMoved, // current distance moved (in odometry units)
	double setPoint, // goal distance to move (in odometry units)
	bool isPositive, // direction of movement
	ConstantContainer constants, // all constants
	PIDReturn lastCycle // data from previous cycle
	)
{
	PIDReturn thisCycle;
	// P: Proportional -- slows down as we reach our target for more accuracy

		// error = goal reading - current reading
		double error = setPoint - distanceMoved;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		double proportionalOut = error * constants.kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		double integral = lastCycle.prevIntegral + error;
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
		double integralOut = integral * constants.kI;

		// adds integral to return structure for compounding
		thisCycle.prevIntegral = integral;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
        double derivative = error - lastCycle.prevError;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        double derivativeOut = derivative * constants.kD;

		// sets the previous error to the current error for use in the next derivative
		thisCycle.prevError = error;



	// Adds the results of each of the calculations together to get the desired power
		double power = proportionalOut + integralOut + derivativeOut;

		thisCycle.power = power;

	// returns a PIDReturn structure
		return thisCycle;
}