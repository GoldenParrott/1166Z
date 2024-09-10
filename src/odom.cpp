#include "init.h"

void initializeRobotOnCoordinate(pros::Rotation *rotational, // parallel rotational sensor
                          pros::Imu *imu1, // first inertial sensor
                          pros::Imu *imu2, // second inertial sensor
                          Coordinate offset, // an ordered pair representing the current 
                                             // location of the robot in relation to the origin
                          int startHeading, // starting heading in relation to the absolute zero heading
                          int quadrant // quadrant of the field in which the robot starts
                        ) 
{
    // switches the x and y values based on the quadrant of the triangle
    switch (quadrant) {
        case 1:
            offset.x = offset.x;
            offset.y = offset.y;
            break;
        case 2:
            offset.x = -offset.x;
            offset.y = offset.y;
            break;
        case 3:
            offset.x = -offset.x;
            offset.y = -offset.y;
            break;
        case 4:
            offset.x = offset.x;
            offset.y = -offset.y;
            break;
    }

    // starting offset distance in centimeters
    double startDistance = std::pow(offset.x, 2) + std::pow(offset.y, 2);

    // gets the centimeter distance moved in a single degree of rotation
    double singleDegree = calculateSingleDegree(2); // 2 is the pre-measured wheel diameter in inches
    // converts the distance in centimeters to degrees of the tracking wheel
    double valueDeg = startDistance / singleDegree;
    // converts the distance in degrees to the distance in centidegrees for the rotational sensor
    double valueRaw = valueDeg * 100;
    // sets the offset rotational sensor value to the rotational sensor value
    rotational->set_position(valueRaw);

    // sets the headings to the heading offset
    imu1->set_heading(startHeading);
    imu2->set_heading(startHeading);
}


Coordinate getLocation(double heading, double dist, Coordinate prevLoc) {
    // switches the heading based on the direction of the turn
    heading = dist > 0
        ? heading // does nothing if the distance moved is positive
        : heading < 180 // flips if the distance moved is negative
            ? heading + 180 // flips the heading to itself + 180 if it is less than 180, putting it on the greater side of the circle
            : heading - 180; // flips the heading to itself - 180 if it is greater than 180, putting it on the lesser side of the circle

    // calculates the angle of only the triangle by subtracting from it based on its quadrant
    int triangleAngle = 0;
    if (heading < 90) {
       triangleAngle = 90 - heading;
    } else if (heading < 180) {
        triangleAngle = heading - 90;
    } else if (heading < 270) {
        triangleAngle = 270 - heading;
    } else {
        triangleAngle = heading - 270;
    }

    // treats the distance moved as the hypotenuse of and the heading as the base angle of a triangle
    // and uses them to calculate the value of both legs (the changes in x and y)
    double xChange = std::cos(heading) * dist;
    double yChange = std::sin(heading) * dist;

    // sets the final x and y positions to the changes in x and y added to the previous coordinates
    double xLoc = prevLoc.x + xChange;
    double yLoc = prevLoc.y + yChange;

    // initialized final coordinate as structure for return
    Coordinate finalCoord;
    finalCoord.x = xLoc;
    finalCoord.y = yLoc;

    return finalCoord;
}
