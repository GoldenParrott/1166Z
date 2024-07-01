#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);
    pros::Controller Partner(pros::E_CONTROLLER_PARTNER);

//Motors

    pros::Motor UpLeft(5,0);
    pros::Motor UpRight(6,1);
    pros::Motor_Group IntakePTO({UpLeft, UpRight});
    
    pros::Motor FrontLeft(1,1);
    pros::Motor BackLeft(2,1);
    pros::Motor_Group LeftWheels({FrontLeft, BackLeft});
    pros::Motor_Group AllLeftWheels({FrontLeft, BackLeft, UpLeft});

    pros::Motor FrontRight(3,0);
    pros::Motor BackRight(4,0);
    pros::Motor_Group RightWheels({FrontRight, BackRight});
    pros::Motor_Group AllRightWheels({FrontRight, BackRight, UpRight});

    pros::Motor_Group AllWheels({FrontLeft, BackLeft, FrontRight, BackRight});
    pros::Motor_Group AllAllWheels({FrontLeft, BackLeft, FrontRight, BackRight, UpLeft, UpRight});

    pros::Motor IntakeLeft(11,1);
    pros::Motor IntakeRight(18,1);
    pros::Motor_Group Intake({IntakeLeft,IntakeRight});

/*
ABCDEFGH
12345678
Pnuematics
*/

    pros::ADIDigitalOut MobileGoalManipulator(1);
    pros::ADIDigitalOut IntakePTOPiston(2);

//Other ADI

    pros::ADIAnalogIn AutonSelect(3);
    pros::ADIDigitalIn LowerLimit(4);

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
bool mobileGoalManipulatorValue = false;
bool armMoving = false;
bool armMovementComplete = false;



#endif