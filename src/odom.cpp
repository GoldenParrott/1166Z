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

    // sets the current location to the offset
    universalCurrentLocation = offset;

    // sets the headings to the heading offset
    imu1->set_heading(startHeading);
    imu2->set_heading(startHeading);
}


Coordinate updateLocation(double heading, double dist, Coordinate prevLoc) {
    // switches the heading based on the direction of the turn
    heading = dist >= 0
        ? heading // does nothing if the distance moved is positive
        : heading < 180 // flips if the distance moved is negative
            ? heading + 180 // flips the heading to itself + 180 if it is less than 180, putting it on the greater side of the circle
            : heading - 180; // flips the heading to itself - 180 if it is greater than 180, putting it on the lesser side of the circle

    // calculates the angle of only the triangle by subtracting from it based on its quadrant
    double triangleAngle = 0;
    if (heading < 90) {
       triangleAngle = heading;
    } else if (heading < 180) {
        triangleAngle = heading - 90;
    } else if (heading < 270) {
        triangleAngle = heading - 180;
    } else if (heading < 360) {
        triangleAngle = 360 - heading;
    }

    // treats the distance moved as the hypotenuse of and the heading as the base angle of a triangle
    // and uses them to calculate the value of both legs (the changes in x and y)
    double xChange = std::sin(triangleAngle) * dist;
    double yChange = std::cos(triangleAngle) * dist;

    // sets the final x and y positions to the changes in x and y added to the previous coordinates
    double xLoc = prevLoc.x + xChange;
    double yLoc = prevLoc.y + yChange;

    return {xLoc, yLoc};
}

// continually updates the value of the universal current location for use by every function
void updateCoordinateLoop() {

    // declaration of previous location
    Coordinate previousLocation = universalCurrentLocation;
    
    while (true) {
        // updates the location
        universalCurrentLocation = updateLocation(getAggregatedHeading(Kalman1, Kalman2), readOdomPod(Rotational), previousLocation);
        // resets the rotational for the next movement
        Rotational.reset_position();
        // previous location for use in next cycle
        previousLocation = universalCurrentLocation;
        // delay between cycles
        pros::delay(5);
    }
}