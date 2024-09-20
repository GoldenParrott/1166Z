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
    double originalHeading = heading;
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
        xChange = std::sin(((triangleAngle * 3.141592) / 180)) * dist;
        yChange = std::cos(((triangleAngle * 3.141592) / 180)) * dist;
    }
    // otherwise, if the heading is in quadrants 2 or 4, then the x-value is the adjacent leg (cosine) and the y-value is the opposite leg (sine)
    else {
        xChange = std::cos(((triangleAngle * 3.141592) / 180)) * dist;
        yChange = std::sin(((triangleAngle * 3.141592) / 180)) * dist;
    }

    // reverses the movement of x and y if they moved in a positive direction and moved down or if they moved in a negative direction and moved up
    // checks for this by checking if the robot moved forward and had a heading that is in the positive y-axis
    if (originalHeading > 90 && originalHeading < 270) { // if heading is between 90 and 270 on the bottom side (moving down in the y-axis), flip the y-movement
        yChange = -yChange;
    }
    if (originalHeading < 360 && originalHeading > 180) { // if heading is between 180 and 360 on the left side (moving down in the x-axis), flip the x-movement
        xChange = -xChange;
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

// this is a reversed version of the update coordinate function that finds the heading of a line on a coordinate plane given two poins on the line
// it is used to find headings for corrective turns
double findHeadingOfLine(
    Coordinate point1, // the initial point
    Coordinate point2 // the final point
)
{
    // finds the difference between the x- and y- values to get the rise and run of the line
    double xChange = point2.x - point1.x;
    double yChange = point2.y - point1.y;


    bool yIsPositive = yChange > 0;
    bool xIsPositive = xChange > 0;

    // finds the positive position of the angle in relation to the previous multiple of 90 degrees
    double triangleAngle = 0;
    if ((yIsPositive && xIsPositive) || (!yIsPositive && !xIsPositive)) { // quadrants 1 or 3
        triangleAngle = std::atan(abs(xChange) / abs(yChange)); // tangent is opposite/adjacent, and x is opposite in these cases
    }
    else { // quadrants 2 or 4
        triangleAngle = std::atan(abs(yChange) / abs(xChange)); // tangent is opposite/adjacent, and y is opposite in these cases
    }

    double heading = 0;
    if (xIsPositive && yIsPositive) { // quadrant 1
       heading = heading;
    } else if (!xIsPositive && yIsPositive) { // quadrant 2
        heading = triangleAngle + 90;
    } else if (!xIsPositive && !yIsPositive) { // quadrant 3
        heading = triangleAngle + 180;
    } else if (xIsPositive && !yIsPositive) { // quadrant 4
        heading = triangleAngle + 270;
    }
    return heading;
}