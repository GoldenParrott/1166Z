#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);

//Motors

    pros::Motor FrontLeft(12, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor MidLeft(1, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor BackLeft(4, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor_Group LeftWheels({FrontLeft, MidLeft, BackLeft});

    pros::Motor FrontRight(20, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor MidRight(7, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor BackRight(17, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor_Group RightWheels({FrontRight, MidRight, BackRight});

    pros::Motor_Group AllWheels({FrontLeft, FrontRight, BackLeft, BackRight, MidLeft, MidRight});

    pros::Motor Preroller(19, pros::E_MOTOR_GEAR_200, 0);
    pros::Motor Transport(8, pros::E_MOTOR_GEAR_200, 1);
    pros::Motor_Group Intake({Preroller, Transport});

    pros::Motor Arm(10, pros::E_MOTOR_GEAR_200, 0);

/*
ABCDEFGH
||||||||
12345678
Pnuematics
*/

    //mogo
         pros::ADIDigitalOut Clamp(1);

    // Rotational Sensors
        pros::Rotation Rotational(9);
        pros::Rotation RotationalTurn(6);
        pros::Rotation ArmRotational(21);

    // Other Sensors
        pros::Optical colorSense(18);
        pros::Distance Distance(15);

    // Inertial Sensors/Kalman Filters
        pros::IMU Inertial1(5);
        KalmanFilter Kalman1 = KalmanFilter(&Inertial1, &RotationalTurn);
        pros::IMU Inertial2(16);
        KalmanFilter Kalman2 = KalmanFilter(&Inertial2, &RotationalTurn);

// Task Pointers

    pros::Task* controlLoop_task_ptr = NULL;
    pros::Task* coordinateUpdater_task_ptr = NULL;
    pros::Task* rotationalBinder_task_ptr = NULL;
    pros::Task* autoSelector_task_ptr = NULL;
      
// Variables

    int num;
    bool clampOn;

    int autonnumber = 0;
    bool globalAuton = true;
    int logoCount = 0;

    int drvtrDZ = 10;
    int drvtrFB;
    int drvtrLR;
    // upright = 1900, upleft = -1900

    bool toggleColorSensor = false;
    int colorDelay = 0;
    bool ejectIsEjecting = false;

    bool presettingA = false;
    bool presettingX = false;

    Point endCoords;
    bool endReverse;
    bool endended;

    Point universalCurrentLocation = {0, 0};

pros::screen_touch_status_s_t status;