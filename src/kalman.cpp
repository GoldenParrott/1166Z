#include "init.h"
// Kalman Filter class methods

// (private)
void KalmanFilter::KalmanFilterLoop()
{

    // current measurement value
    int currentMeasurement = 0;

    // current actual values
    int currentHeading;
    int currentCovariance; // initial guess

    // predicted measurement values for later in the cycle
    int statePrediction = 0;
    int estimateVariancePrediction = 0;

    // variable for the Kalman gain
    int kalmanGain = 0;

    // standard deviation of the variances
    int measurementDeviation = 0;
    int predictionDeviation = 0;

    // velocity values
    double velocity = 0;
    pros::c::imu_accel_s_t rawAcceleration;
    double msAcceleration = 0;
    // direction of accelerometer values for velocity
    bool isPositive = true;
    bool wasPositive = true;
            
    // initial loop

    // measurement guess phase
    currentHeading = inertial->get_heading(); // initial measurement, treated as "filtered" for starting cycle
    currentCovariance = 0; // initial guess

    // prediction phase
    statePrediction = currentHeading + (velocity * (this->delay / 1000));; // State Extrapolation Equation
    estimateVariancePrediction = currentCovariance + (std::pow((this->delay / 1000), 2) * predictionDeviation); // Covariance Extrapolation Equation

    // looping filter
    while (true) {
                // MEASUREMENT PHASE
                currentMeasurement = inertial->get_heading();

                // KALMAN GAIN CALCULATION
                kalmanGain = estimateVariancePrediction / (estimateVariancePrediction + measurementDeviation); // Kalman Gain Equation

                // UPDATE PHASE
                currentHeading = statePrediction + (kalmanGain * (currentMeasurement - statePrediction)); // State Update Equation
                currentCovariance = (1 - kalmanGain) * estimateVariancePrediction; // Covariance Update Equation

                // VELOCITY UPDATE
                pros::c::imu_accel_s_t rawAcceleration = inertial->get_accel(); // gets the raw accelerometer values

                rawAcceleration.x *= 9.807; // changes the units of x from G's to m/s
                rawAcceleration.z *= 9.807; // changes the units of x from G's to m/s
                
                msAcceleration = (abs(rawAcceleration.x) + abs(rawAcceleration.z)) / 2; // sets the acceleration value to the mean 
                                                                                        // of the x and y accelerometer values (always positive)

                isPositive = msAcceleration > 0; // gives a boolean value that is equal to the sign of the acceleration 
                                                 // for the current cycle (positive = true, negative = false)

                // checks if the direction of acceleration has switched and switches the direction of the msAcceleration value if it has
                if (isPositive = !wasPositive) {
                    msAcceleration *= -1; // switches the direction of acceleration 
                                // (multiplying by -1 switches a positive to a negative and a negative to a positive)
                }

                wasPositive = msAcceleration > 0; // gives a boolean value that is equal to the sign of the acceleration 
                                                 // for the next cycle (positive = true, negative = false)
    
                velocity += msAcceleration * (this->delay / 1000); // updates the velocity of the robot





                pros::Controller Master(pros::E_CONTROLLER_MASTER);
                Master.print(0, 0, "X = %f", msAcceleration);

                // VARIANCE/DEVIATION CALCULATION

                // measurement variance
                measurementVariances.push_back(currentMeasurement - currentHeading);
                measurementDeviation = calculateStandardDeviation(measurementVariances);
                // prediction variance
                predictionVariances.push_back(statePrediction - currentHeading);
                predictionDeviation = calculateStandardDeviation(predictionVariances);

                // PREDICTION PHASE
                statePrediction = currentHeading + (velocity * (this->delay / 1000));; // State Extrapolation Equation
                estimateVariancePrediction = measurementDeviation + (std::pow((this->delay / 1000), 2) * predictionDeviation); // Covariance Extrapolation Equation

                // ENDING DELAY AND OUTPUT UPDATE
                this->filteredHeading = currentHeading;
                this->filterUncertainty = currentCovariance;
                pros::delay(this->delay);
    }
}



// calculates the standard deviation of a set of values, used for the filter (private)
double KalmanFilter::calculateStandardDeviation(
    std::vector<double> listOfDifferences // list of differences from the estimates
)
{
    double standardDeviation = 0;

    for (int i = 0; i < listOfDifferences.size(); i++) {
        standardDeviation += std::pow(listOfDifferences[i], 2); // squares each value's distance from its estimate to find the standard deviation
    }

    standardDeviation = standardDeviation / listOfDifferences.size();
    return standardDeviation;
}



// constructor (public)
KalmanFilter::KalmanFilter(pros::IMU* inertial, pros::Rotation* turnRotational) {

    // instance variable initializations
    filterLoop_ptr = NULL; // sets the filter loop pointer to null so it can be defined later
    filteredHeading = -1; // filtered heading is set to -1 as an unset value
    filterUncertainty = -1; // filtered uncertainty is set to -1 as an unset value

    delay = 5; // delay (in ms) between cycles

    this->inertial = inertial; // takes the IMU for its heading
    this->turnRotational = turnRotational; // takes the turning rotation
}

/* 
* returns the filtered heading that the filter provides,
* but will return -1 if the filter is not active
* (public)
*/
int KalmanFilter::getFilteredHeading()
    {return this->filteredHeading;}

// (public)
int KalmanFilter::getFilterUncertainty()
    {return this->filterUncertainty;}

// starts the filter loop if it is not already active (public)
void KalmanFilter::startFilter() {

    auto filterLoopFunction = [this]() {return this->KalmanFilterLoop();};

    if (filterLoop_ptr == NULL) {
        filterLoop_ptr = new pros::Task(filterLoopFunction);
    }
}

// completely ends the filter loop if it is active (public)
void KalmanFilter::endFilter() {
    if (filterLoop_ptr != NULL) {

        filterLoop_ptr->remove();
        filterLoop_ptr = NULL;

        filteredHeading = -1;

        measurementVariances.clear();
        predictionVariances.clear();
    }
}