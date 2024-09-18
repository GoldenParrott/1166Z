#include "init.h"

void initializeRobotOnCoordinate(pros::Rotation *rotational, // parallel rotational sensor
                          pros::Imu *imu1, // first inertial sensor
                          pros::Imu *imu2, // second inertial sensor
                          Coordinate offset, // an ordered pair representing the current 
                                             // location of the robot in relation to the origin
                          int startHeading // starting heading in relation to the absolute zero heading
                        ) 
{
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
        triangleAngle = heading - 270;
    }
    pros::lcd::print(3, "H = %f", triangleAngle);
    // treats the distance moved as the hypotenuse of and the heading as the base angle of a triangle
    // and uses them to calculate the value of both legs (the changes in x and y)
    double xChange = 0;
    double yChange = 0;
    // if the heading is in quadrants 1 or 3, then the x-value is the opposite leg (sine) and the y-value is the adjacent leg (cosine)
    if ((heading < 90) || (heading >= 180 && heading < 270)) {
        xChange = std::sin(((triangleAngle * 3.14) / 180)) * dist;
        yChange = std::cos(((triangleAngle * 3.14) / 180)) * dist;
    }
    // otherwise, if the heading is in quadrants 2 or 4, then the x-value is the adjacent leg (cosine) and the y-value is the opposite leg (sine)
    else {
        xChange = std::cos(((triangleAngle * 3.14) / 180)) * dist;
        yChange = std::sin(((triangleAngle * 3.14) / 180)) * dist;
    }

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