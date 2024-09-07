#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);

//Motors
    
    pros::Motor FrontLeft(99, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor MidLeft(99, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor BackLeft(99, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor_Group LeftWheels({FrontLeft, MidLeft, BackLeft});

    pros::Motor FrontRight(99, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor MidRight(14, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor BackRight(99, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor_Group RightWheels({FrontRight, MidRight, BackRight});

    pros::Motor_Group AllWheels({FrontLeft, MidLeft, BackLeft, FrontRight, MidRight, BackRight});

    pros::Motor InputMotor(99, pros::E_MOTOR_GEAR_200, 0);
    pros::Motor Transport(99, pros::E_MOTOR_GEAR_200, 1);
    pros::Motor_Group Intake({InputMotor, Transport});

    pros::Motor Arm(99, pros::E_MOTOR_GEAR_200, 1);

/*
ABCDEFGH
||||||||
12345678
Pnuematics
*/

    pros::ADIDigitalOut MobileGoalManipulator(99);
    pros::ADIDigitalOut ArmPiston(99);
    pros::ADIDigitalOut Grabber(99);


// Tasks

    pros::Task* colorSensorOn_task_ptr = NULL;


// Sensors

    pros::Optical colorSense(99);
    pros::IMU Inertial1(99);
    pros::IMU Inertial2(99);
    pros::Rotation Rotational(99, 1);
    pros::Rotation RotationalTurn(99);

//Variables
int autonnumber;
int logoCount = 0;

int drvtrDZ = 10;
int drvtrFB;
int drvtrLR;
// upright = 1900, upleft = -1900

bool toggleColorSensor = false;
int colorDelay = 0;

bool redirectOn = false;
int redirectStartPoint = 0;

bool presettingA = false;
bool presettingX = false;