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

Coordinate getLocation(double heading, double dist) {
    // ensures that the heading is between 0 and 90 so it can be treated as an angle of a right triangle
    int quadrant;
    if (heading < 90) {
       
    } else if (heading < 180) {
        heading -= 90;
    } else if (heading < 270) {
        heading -= 180;
    } else {
        heading -= 270;
    }
    // treats the distance moved as the hypotenuse of and the heading as the base angle of a triangle
    // and uses them to calculate the value of both legs (the x and y locations)
    double xLoc = std::cos(heading) * dist;
    double yLoc = std::sin(heading) * dist;

    return Coordinate(xLoc, yLoc);
}
