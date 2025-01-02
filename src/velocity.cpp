#include "init.h"

VelocityController::VelocityController(double wheelDiameter, double distBetweenWheels, double gearRatio, double maxRPM) {
    this->wheelDiameter = wheelDiameter;
    this->distBetweenWheels = distBetweenWheels;
    this->gearRatio = gearRatio;
    this->maxRPM = maxRPM;
}

// uses the kinematic equations of a differential chassis and unit conversions to convert a linear and angular velocity to something that can be used
// by each side of the drivetrain
std::vector<double> VelocityController::calculateOutputOfSides(double linearVelocityMPS, double angularVelocityRADPS, Direction direction) {
    
    double leftVelocityMPS = linearVelocityMPS - ((angularVelocityRADPS * (this->distBetweenWheels * 0.0254)) / 2); // lv = v - ((w * L) / 2)
    double rightVelocityMPS = linearVelocityMPS + ((angularVelocityRADPS * (this->distBetweenWheels * 0.0254)) / 2); // rv = v + ((w * L) / 2)

    double leftVelocityRPM = (leftVelocityMPS * 60 / (3.141592 * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double rightVelocityRPM = (rightVelocityMPS * 60 / (3.141592 * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))

    if (leftVelocityRPM > this->maxRPM) {
        double scaling = maxRPM / leftVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    }

    if (rightVelocityRPM > this->maxRPM) {
        double scaling = maxRPM / rightVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    }

    if (((leftVelocityRPM > rightVelocityRPM) && (direction == RIGHT)) ||
    ((rightVelocityRPM > leftVelocityRPM) && (direction == LEFT)))
    {
        double rvcache = rightVelocityRPM;
        rightVelocityRPM = leftVelocityRPM;
        leftVelocityRPM = rvcache;
    }

    return {leftVelocityRPM, rightVelocityRPM};
}

// calculates the linear travel of a single degree of movement for a wheel of a given diameter
double VelocityController::calculateSingleDegree(double wheelDiameter) {
    // sets up the odometry to convert angle readings to cm
    double wheelCircumference = 3.141592 * wheelDiameter; // 2 is the pre-measured wheel diameter in inches
	long double singleDegree = wheelCircumference / 360; // the distance that the robot moves in one degree of rotation of its wheels

    return singleDegree;
}

// (private)
void VelocityController::followProfile(MotionProfile* profile, bool RAMSETE)
{
    double currentStep = 0;
    double len = this->queuedProfile->profile.size();
    double step = 1 / len;
    int x = 0;
    // control loop
    while (true) {
        MPPoint currentPoint = profile->findNearestPoint(currentStep);

        // standard calculation of output of each side based on specifications of the motion profile
        std::vector<double> velocitiesRPM = this->calculateOutputOfSides(currentPoint.linVel, currentPoint.angVel, this->queuedProfile->findCurveDirectionOfPoint(currentPoint));

        // calculation of output of each side with error corrections from RAMSETE
        if (RAMSETE) {
            fixAngle(currentPoint.heading);
        // rotation of the x, y, and heading errors to fit the local frame
            Pose error;
            error.x = (std::cos(currentPoint.heading) * (currentPoint.x - universalCurrentLocation.x)) + (std::sin(currentPoint.heading) * (currentPoint.y - universalCurrentLocation.y));
            error.y = (-1 * std::sin(currentPoint.heading) * (currentPoint.x - universalCurrentLocation.x) + (std::cos(currentPoint.heading) * (currentPoint.y - universalCurrentLocation.y)));
            error.heading = currentPoint.heading - getAggregatedHeading(Kalman1, Kalman2);

        // tuning constants (current values are from the widely accepted defaults from FTCLib)
            double b = 2.0; // this is a proportional gain for each of the different error elements of the controller (put into the gain value calculations)
            double zeta = 0.6; // this is a dampener for the direct movements (k1 and k3/x-value and heading-value)

        // gain values that serve as scaling multipliers for the outputs based on the profile's velocity and pre-set constants
            // k1/k3: the proportional gain value for both the local frame of x and heading
            double k = 2 * zeta * std::sqrt(std::pow(currentPoint.angVel, 2) + (b * std::pow(currentPoint.linVel, 2)));
            // k2: the gain value for the y-value, which the robot cannot move directly on and is thus handled differently 
            double k2 = b * currentPoint.linVel;

        // control inputs that determine the controller's influence on the velocity based on the error
            // simply the normal gain value multiplied by the x-error because that can be directly moved upon (input for linear velocity)
            double u1 = -1 * k * error.x;
            // the special gain value is used for the y-value, and it is also scaled with the part in purple parenthesis to let it switch directions smoothly if it is past
            // its goal point; the second part is the same simple direct angular movement for the error in heading that is used in the same way with the x-value for linear
            // movement
            double u2 = (-1 * k2 * (std::sin(error.heading) / error.heading) * error.y) + (-1 * k * error.heading);

        // actual calculations of modified linear and angular velocities as additions/subtractions to the profile's original values
            // the original linear velocity is also transformed to follow the robot's error in heading before having the control input subtracted from it
            double linVel = (currentPoint.linVel * std::cos(error.heading)) - u1;
            // the angular velocity does not need to be transformed in the same way that the linear velocity needs to because it is already angular in reference to the robot
            double angVel = currentPoint.angVel - u2;
        }

        auto RPMtoMPS = [] (double gearset, double gearRatio, double diameter) {
            return (gearset * gearRatio * (3.14 * diameter)) / 60;
        };
        MPPoint nextPoint = this->queuedProfile->profile[(currentStep + step) * 512];
        double timeAtCurrentVelocity = (calculateDistance({currentPoint.x, currentPoint.y}, {nextPoint.x, nextPoint.y}) / 100) / ((RPMtoMPS(velocitiesRPM[0], gearRatio, wheelDiameter) + (RPMtoMPS(velocitiesRPM[1], gearRatio, wheelDiameter))) / 2);

        for (int i = 0; i < actions.size(); i++) {
            if (currentPoint.t == actionTs[i]) {
                actions[i]();
            }
        }

        LeftWheels.move_velocity(velocitiesRPM[0]);
        RightWheels.move_velocity(velocitiesRPM[1]);


        std::cout << "lrpm = " << velocitiesRPM[0] << ", rrpm = " << velocitiesRPM[1] << "\n";
        std::cout << "x = " << currentPoint.x << ", y = " << currentPoint.y << "\n";
        std::cout << "step = " << currentStep << "\n";
        std::cout << " lvel = " << currentPoint.linVel << ", avel = " << currentPoint.angVel << "\n\n";
        
        pros::delay(timeAtCurrentVelocity / 1000);

        currentStep += step;

        if (currentPoint.t == 1) {
            AllWheels.brake();
            return;
        }
    }
}

// starts the filter loop if it is not already active (public)
void VelocityController::startQueuedProfile(bool RAMSETE) {


        auto controlLoopFunction = [this, RAMSETE] () {return this->followProfile(this->queuedProfile, RAMSETE);};

    if (controlLoop_task_ptr == NULL) {
        controlLoop_task_ptr = new pros::Task(controlLoopFunction);
    }
}

// completely ends the filter loop if it is active (public)
void VelocityController::endProfile() {
    if (controlLoop_task_ptr != NULL) {
        controlLoop_task_ptr->remove();
        controlLoop_task_ptr = NULL;
    }
}

void VelocityController::queueProfile(MotionProfile* profile) {
    this->queuedProfile = profile;
}

void VelocityController::addAction(std::function<void(void)> action, double time) {
    double t = time / this->timeToRun;
    double actionT = this->queuedProfile->findNearestPoint(t).t;
    this->actions.push_back(action);
    this->actionTs.push_back(actionT);
}