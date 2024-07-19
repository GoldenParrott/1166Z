#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);
    pros::Controller Partner(pros::E_CONTROLLER_PARTNER);

//Motors

    pros::Motor UpLeft(13, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor UpRight(19, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor_Group IntakePTO({UpLeft, UpRight});
    
    pros::Motor FrontLeft(11, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor BackLeft(5, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor_Group LeftWheels({FrontLeft, BackLeft});
    pros::Motor_Group AllLeftWheels({FrontLeft, BackLeft, UpLeft});

    pros::Motor FrontRight(1, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor BackRight(2, pros::E_MOTOR_GEAR_600, 0);
    pros::Motor_Group RightWheels({FrontRight, BackRight});
    pros::Motor_Group AllRightWheels({FrontRight, BackRight, UpRight});

    pros::Motor_Group AllWheels({FrontLeft, BackLeft, FrontRight, BackRight});
    pros::Motor_Group AllAllWheels({FrontLeft, BackLeft, FrontRight, BackRight, UpLeft, UpRight});

    pros::Motor InputMotor(14, pros::E_MOTOR_GEAR_600, 1);
    pros::Motor Transport(3, pros::E_MOTOR_GEAR_200, 1);
    pros::Motor_Group Intake({Transport, InputMotor});

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


//Other ADI



// Sensors

    pros::Optical colorSense(16);
    pros::IMU Inertial(6);

//Variables
int autonnumber;

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