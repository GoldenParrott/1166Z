#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);

//Motors

    pros::Motor FrontLeft(7, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor MidLeft(5, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor BackLeft(2, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor_Group LeftWheels({FrontLeft, MidLeft, BackLeft});

    pros::Motor FrontRight(8, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor MidRight(14, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor BackRight(15, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor_Group RightWheels({FrontRight, MidRight, BackRight});

    pros::Motor_Group AllWheels({FrontLeft, FrontRight, BackLeft, BackRight, MidLeft, MidRight});

    pros::Motor InputMotor(11, pros::E_MOTOR_GEAR_200, 0);
    pros::Motor Transport(4, pros::E_MOTOR_GEAR_200, 1);
    pros::Motor_Group Intake({InputMotor, Transport});

    pros::Motor Arm(13, pros::E_MOTOR_GEAR_200, 1);

/*
ABCDEFGH
||||||||
12345678
Pnuematics
*/

    pros::ADIDigitalOut MobileGoalManipulator(8);
    pros::ADIDigitalOut InputPiston(6);
    pros::ADIDigitalOut ArmPiston(5);
    pros::ADIDigitalOut Grabber(7);
    pros::ADIDigitalOut Hang(4);


// Tasks

    pros::Task* coordinateUpdater_task_ptr = NULL;
    pros::Task* rotationalBinder_task_ptr = NULL;
    pros::Task* autoSelector_task_ptr = NULL;

// Sensors

    pros::Optical colorSense(1);
    pros::Distance Distance(19);

    pros::Rotation Rotational(12);
    pros::Rotation RotationalTurn(20);

    pros::IMU Inertial1(6);
    KalmanFilter Kalman1 = KalmanFilter(&Inertial1, &RotationalTurn);
    pros::IMU Inertial2(9);
    KalmanFilter Kalman2 = KalmanFilter(&Inertial2, &RotationalTurn);

//Variables
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

Coordinate universalCurrentLocation = {0, 0};

pros::screen_touch_status_s_t status;