#include "init.h"

double readOdomPod(pros::Rotation odomRotational) {
    // sets up the odometry to convert angle readings to m/s
    double wheelCircumference = 3.14 * 2; // 2 is the wheel diameter in inches
	double wheelRevolution = wheelCircumference * 2.54; // wheel circumference in cm
						// this is equivalent to how far the robot moves in one 360-degree rotation of its wheels
	long double singleDegree = wheelRevolution / 360; // the distance that the robot moves in one degree of rotation of its wheels


    // gets the reading from the rotational sensor
    int rawReading = odomRotational.get_position(); // gives centidegrees (a 0-36,000 scale)
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale)
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return readingCM;
}


double readOdomVelocity(pros::Rotation odomRotational) {
    // sets up the odometry to convert angle readings to m/s
    double wheelCircumference = 3.14 * 2; // 2 is the wheel diameter in inches
	double wheelRevolution = wheelCircumference * 2.54; // wheel circumference in cm
						// this is equivalent to how far the robot moves in one 360-degree rotation of its wheels
	long double singleDegree = wheelRevolution / 360; // the distance that the robot moves in one degree of rotation of its wheels


    // gets the reading from the rotational sensor
    int rawReading = odomRotational.get_velocity(); // gives centidegrees (a 0-36,000 scale) per second
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale) per second
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return readingCM;
}