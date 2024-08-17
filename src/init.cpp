#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);
    pros::Controller Partner(pros::E_CONTROLLER_PARTNER);

//Motors
    
    pros::Motor FrontLeft(99, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor MidLeft(99, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor BackLeft(99, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor_Group LeftWheels({FrontLeft, MidLeft, BackLeft});

    pros::Motor FrontRight(99, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor MidRight(99, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor BackRight(99, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor_Group RightWheels({FrontRight, MidRight, BackRight});

    pros::Motor_Group AllWheels({FrontLeft, MidLeft, BackLeft, FrontRight, MidRight, BackRight});

    pros::Motor InputMotor(16, pros::E_MOTOR_GEAR_200, 0);
    pros::Motor Transport(6, pros::E_MOTOR_GEAR_200, 1);
    pros::Motor_Group Intake({InputMotor, Transport});

    pros::Motor ArmLeft(99, pros::E_MOTOR_GEAR_200, 0);
    pros::Motor ArmRight(99, pros::E_MOTOR_GEAR_200, 1);
    pros::Motor_Group Arm({ArmLeft, ArmRight});

/*
ABCDEFGH
12345678
Pnuematics
*/

    pros::ADIDigitalOut MobileGoalManipulator(5);
    pros::ADIDigitalOut IntakePTOPiston(1);
    pros::ADIDigitalOut InputPiston(2);
    pros::ADIDigitalOut Eject(8);
    pros::ADIDigitalOut GrabPiston(4);


// Tasks

    pros::Task* colorSensorOn_task_ptr = NULL;


// Sensors

    pros::Optical colorSense(16);
    pros::IMU Inertial(6);
    pros::Rotation Rotational(99);
    pros::Rotation RotationalTurn(99);

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