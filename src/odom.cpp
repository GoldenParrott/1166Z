#include "init.h"

double readOdomPod(pros::Rotation odom) {
    // sets up the odometry to convert angle readings to m/s
    double wheelCircumference = 3.14 * 2; // 2 is the pre-measured wheel diameter in inches
	double wheelRevolution = wheelCircumference * 2.54; // wheel circumference in cm
						// this is equivalent to how far the robot moves in one 360-degree rotation of its wheels
	long double singleDegree = wheelRevolution / 360; // the distance that the robot moves in one degree of rotation of its wheels


    // gets the reading from the rotational sensor
    int rawReading = odom.get_position(); // gives centidegrees (a 0-36,000 scale)
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale)
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return readingCM;
}


double readOdomVelocity(pros::Rotation odom) {
    // sets up the odometry to convert angle readings to m/s
    double wheelCircumference = 3.14 * 2; // 2 is the wheel diameter in inches
	double wheelRevolution = wheelCircumference * 2.54; // wheel circumference in cm
						// this is equivalent to how far the robot moves in one 360-degree rotation of its wheels
	long double singleDegree = wheelRevolution / 360; // the distance that the robot moves in one degree of rotation of its wheels


    // gets the reading from the rotational sensor
    int rawReading = odom.get_velocity(); // gives centidegrees (a 0-36,000 scale) per second
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale) per second
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return rawReading;
}

double readOdomAngle(pros::Rotation turnOdom) {
    double robotHeading = turnOdom.get_angle() / 100;

    return robotHeading;
}











double getAggregatedHeading(KalmanFilter inertial1, KalmanFilter inertial2, pros::Rotation turnOdom) {
    // gets heading from each of the sensors
    double I1Heading = inertial1.getFilteredHeading();
    double I2Heading = inertial1.getFilteredHeading();
    double OdomHeading = readOdomAngle(turnOdom);

    // gets the uncertainty from each of the IMUs
    double I1Uncertainty = inertial1.getFilterUncertainty();
    double I2Uncertainty = inertial2.getFilterUncertainty();
    double OdomUncertainty = 0.3; // pre-set by user

    // weights each of the IMUs
    double I1Weight = OdomUncertainty / (OdomUncertainty + I1Uncertainty); // weight of I1
    double I2Weight = OdomUncertainty / (OdomUncertainty + I2Uncertainty); // weight of I2
    if (I1Weight + I2Weight > 1) { // only triggers if the weight of both values is over 1
        double excess = (I1Weight + I2Weight) - 1; // calculates the excess by taking the combination 
                                                   // of both values and subtracting 1 from them

        // removes the excess by removing half of it from each weight, in total removing all of it
        I1Weight -= (excess / 2);
        I2Weight -= (excess / 2);
    }

    double OdomWeight = 1 - (I1Uncertainty + I2Uncertainty); // weight of odometry pod (lesser if the uncertainties of the other values are smaller)

    // aggregates the heading by applying the weights to all the headings
    double aggregatedHeading = (I1Weight * I1Heading) + (I2Weight * I2Heading) + (OdomWeight * OdomHeading);

    return aggregatedHeading;
}