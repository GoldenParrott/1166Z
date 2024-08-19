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
                currentMeasurement = inertial->get_heading(); // unit is degrees

                // KALMAN GAIN CALCULATION
                kalmanGain = estimateVariancePrediction / (estimateVariancePrediction + measurementDeviation); // Kalman Gain Equation (value = deg / (deg + deg))

                // UPDATE PHASE
                currentHeading = statePrediction + (kalmanGain * (currentMeasurement - statePrediction)); // State Update Equation (deg = deg + (value * (deg - deg)))
                currentCovariance = (1 - kalmanGain) * estimateVariancePrediction; // Covariance Update Equation

                // VELOCITY UPDATE
                velocity = readOdomVelocity(*turnRotational);

                // VARIANCE/DEVIATION CALCULATION

                // measurement variance
                measurementVariances.push_back(currentMeasurement);
                if (measurementVariances.size() > 50) {measurementVariances.pop_front();}
                measurementDeviation = calculateStandardDeviation(measurementVariances); // unit is degrees
                // prediction variance
                predictionVariances.push_back(statePrediction);
                if (measurementVariances.size() > 50) {measurementVariances.pop_front();}
                predictionDeviation = calculateStandardDeviation(predictionVariances); // unit is degrees

                // PREDICTION PHASE
                statePrediction = currentHeading + (velocity * (this->delay / 1000));; // State Extrapolation Equation (deg = deg + (dps * s))
                estimateVariancePrediction = currentCovariance + (std::pow((this->delay / 1000), 2) * predictionDeviation); // Covariance Extrapolation Equation (deg = deg + (sec^2 * deg))

                // ENDING DELAY AND OUTPUT UPDATE
                this->filteredHeading = measurementDeviation;
                this->filterUncertainty = currentCovariance;
                pros::delay(this->delay);
    }
}



// calculates the standard deviation of a set of values, used for the filter (private)
double KalmanFilter::calculateStandardDeviation(
    std::deque<double> listOfValues // list of differences from the estimates
)
{
    // calculates the mean of the values
    double meanOfValues = 0;
    for (int i = 0; i < listOfValues.size(); i++) {
        meanOfValues += listOfValues[i]; // adds each value together to calculate their mean
    }
    meanOfValues = meanOfValues / listOfValues.size();

    // calculates the variance of the values
    std::vector<double> listOfDifferences;
    double variance = 0;
    for (int i = 0; i < listOfValues.size(); i++) {
        variance += std::pow((listOfValues[i] - meanOfValues), 2); // variance is calculated by adding the squares of each value's difference from the mean together
    }
    variance = variance / listOfValues.size();

    double standardDeviation = std::sqrt(variance); // standard deviation = square root of variance

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