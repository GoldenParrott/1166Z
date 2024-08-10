#include "init.h"


class KalmanIMU {

    private:
        // instance variables
        pros::IMU* sensor;
        int filteredHeading = -1;
        pros::Task* filterLoop_ptr = NULL;

        // constructor
        KalmanIMU(pros::IMU* sensor) {
            this->sensor = sensor;
        }


        int KalmanFilterLoop()
        {
            int measurement = Inertial.get_heading();
        }


        double calculateKalmanGain(
            double prediction, // the prediction for the current cycle from the last cycle
            double measurementUncertainty // the uncertainty of the inertial reading
        ) 
        {
            double kalmanGain = prediction / (prediction + measurementUncertainty);
            return kalmanGain;
        }




    public:
        /* 
        * returns the filtered heading that the filter provides,
        * but will return -1 if the filter is not active
        */
        int getFilteredHeading(
            int inertialID // the ID of the previously registered inertial sensor
        )
        {return filteredHeading;}

        // starts the filter loop if it is not already active
        void startFilter() {
            if (filterLoop_ptr == NULL) {
                filterLoop_ptr = new pros::Task(this->KalmanFilterLoop());
            }
        }

        // ends the filter loop if it is active
        void endFilter() {
            if (filterLoop_ptr != NULL) {
                filterLoop_ptr->remove();
                filterLoop_ptr = NULL;
                filteredHeading = -1;
            }
        }

};