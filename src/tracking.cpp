#include "init.h"

double calculateSingleDegree(double wheelDiameter) {
    // sets up the odometry to convert angle readings to cm
    double wheelCircumference = 3.141592 * wheelDiameter; // 2 is the pre-measured wheel diameter in inches
	long double singleDegree = wheelCircumference / 360; // the distance that the robot moves in one degree of rotation of its wheels

    return singleDegree;
}





double readOdomPod(pros::Rotation odom) {
    // gets the centimeter distance moved in a single degree of rotation
    double singleDegree = calculateSingleDegree(2.015); // 2 is the pre-measured wheel diameter in inches (may need to be changed for precision)

    // gets the reading from the rotational sensor
    int rawReading = odom.get_position() * -1; // gives centidegrees (a 0-36,000 scale)
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale)
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return readingCM;
}


double readOdomVelocity(pros::Rotation odom) {
    // gets the centimeter distance moved in a single degree of rotation
    double singleDegree = calculateSingleDegree(2); // 2 is the pre-measured wheel diameter in inches


    // gets the reading from the rotational sensor
    int rawReading = odom.get_velocity(); // gives centidegrees (a 0-36,000 scale) per second
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale) per second
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return rawReading;
}

double readOdomAngle(pros::Rotation turnOdom) {
    // the pre-measured distance between the robot's center of rotation
    double distanceBetweenCenterAndOdom = 10.0;

    // gets the distance that the robot moved (in cm)
    double odomReading = (double) readOdomPod(turnOdom);

    // the angle that the robot has moved
    /* measured by finding the central angle of the arc with the distance from the odom pod
     * (using a derived version of the arc formula) */
    double robotHeadingRadians = (double) odomReading / distanceBetweenCenterAndOdom;
    //     theta/angle  =    arc      /           radius
    double robotHeadingDegrees = (robotHeadingRadians * 180.0) / 3.14;

    return robotHeadingDegrees;
}


double getAggregatedHeading(KalmanFilter inertial1, KalmanFilter inertial2) {
    // gets heading from each of the sensors
    double I1Heading = inertial1.getFilteredHeading();
    double I2Heading = inertial2.getFilteredHeading();

    // gets the uncertainty from each of the IMUs
    double I1Uncertainty = inertial1.getFilterUncertainty();
    double I2Uncertainty = inertial2.getFilterUncertainty();

    // weights each of the IMUs
    double I1Weight = I2Uncertainty / (I2Uncertainty + I1Uncertainty); // weight of I1
    double I2Weight = 1 - I1Weight; // weight of I2

    // aggregates the heading by applying the weights to all the headings
    double aggregatedHeading = (I1Weight * I1Heading) + (I2Weight * I2Heading);


    if (abs(I1Heading - I2Heading) > 15) {
        if (I1Uncertainty < I2Uncertainty) {
            aggregatedHeading = I1Heading;
        } else {
            aggregatedHeading = I2Heading;
        }
    }

    return aggregatedHeading;
}


// ensures that the rotational sensor's position is within the range of a heading at all times
void bindTurnTrackingWheelHeading() {
	while (true) {
		if (readOdomAngle(RotationalTurn) > 360) {
			RotationalTurn.set_position(0);
		} else if (readOdomAngle(RotationalTurn) < 0) {
			RotationalTurn.set_position(36000);
		}
		pros::delay(5);
	}
}