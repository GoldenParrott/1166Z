#include "init.h"

void initializeRobotOnCoordinate(pros::Rotation *rotational, // parallel rotational sensor
                          pros::Imu *imu1, // first inertial sensor
                          pros::Imu *imu2, // second inertial sensor
                          Point offset, // an ordered pair representing the current 
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


Point updateLocation(double heading, double dist) {
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
    double xLoc = universalCurrentLocation.x + xChange;
    double yLoc = universalCurrentLocation.y + yChange;

    return {xLoc, yLoc};
}

// continually updates the value of the universal current location for use by every function
void updateCoordinateLoop() {

    // declaration of previous location
    Point previousLocation = universalCurrentLocation;
    // calculates the change in odometry for the location update
    double changeInOdom = 0;
    double previousOdom = 0;
    double cumulativeOdom = 0;
    
    while (true) {
        if (!pros::Task::notify_take(true, 5)) { // while task is not paused by notification, run cycle 
                                                // (the 5 waits for 5 milliseconds before the loop starts and serves as the delay)
            // calculates the current distance moved
            cumulativeOdom = readOdomPod(Rotational);
            // calculates the change in odometry reading based on the previous measurement
            changeInOdom = cumulativeOdom - previousOdom;
            // updates the location
            universalCurrentLocation = updateLocation(getAggregatedHeading(Kalman1, Kalman2), changeInOdom);
            // previous location for use in next cycle
            previousLocation = universalCurrentLocation;
            // cumulative odometry value for use in next cycle as previous value
            previousOdom = cumulativeOdom;
        } else { // ensures that the code does not break while it is paused by a notification
            previousOdom = cumulativeOdom;
            previousLocation = universalCurrentLocation;
        }
    }
}