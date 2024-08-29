#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);

//Motors
    
    pros::Motor FrontLeft(12, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor MidLeft(14, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor BackLeft(11, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor_Group LeftWheels({FrontLeft, MidLeft, BackLeft});

    pros::Motor FrontRight(17, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor MidRight(19, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor BackRight(18, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor_Group RightWheels({FrontRight, MidRight, BackRight});

    pros::Motor_Group AllWheels({FrontLeft, MidLeft, BackLeft, FrontRight, MidRight, BackRight});

    pros::Motor InputMotor(16, pros::E_MOTOR_GEAR_200, 0);
    pros::Motor Transport(7, pros::E_MOTOR_GEAR_200, 1);
    pros::Motor_Group Intake({InputMotor, Transport});

    pros::Motor Arm(6, pros::E_MOTOR_GEAR_200, 1);

/*
ABCDEFGH
||||||||
12345678
Pnuematics
*/

    pros::ADIDigitalOut MobileGoalManipulator(1);
    pros::ADIDigitalOut InputPiston(99);
    pros::ADIDigitalOut ArmPiston(2);
    pros::ADIDigitalOut Grabber(99);


// Tasks

    pros::Task* colorSensorOn_task_ptr = NULL;


// Sensors

    pros::Optical colorSense(99);
    pros::IMU Inertial1(4);
    pros::IMU Inertial2(5);
    pros::Rotation Rotational(99);
    pros::Rotation RotationalTurn(9);

//Variables
int autonnumber = 1;
int logoCount = 0;

int drvtrDZ = 10;
int drvtrFB;
int drvtrLR;
double armGoal;
double armmax;
double armPosition;
// upright = 1900, upleft = -1900

bool armCalibrated = false;
bool intakePTOvalue = false;
bool armMoving = false;
bool armMovementComplete = false;
bool toggleColorSensor = false;
int colorDelay = 0;

bool presettingA = false;
bool presettingX = false;