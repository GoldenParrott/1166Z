#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "api.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);
    pros::Controller Partner(pros::E_CONTROLLER_PARTNER);

//Motors

    pros::Motor UpLeft(5,1);
    pros::Motor UpRight(6,0);
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

    pros::Motor IntakeLeft(7,0);
    pros::Motor IntakeRight(8,1); //Not in use
    pros::Motor_Group Intake({IntakeLeft,IntakeRight});


/*
ABCDEFGH
12345678
Pnuematics*/

    pros::ADIDigitalOut IntakePTOPiston(1);
    pros::ADIDigitalOut Mgm(2);

//Variables
int autonnumber;

int drvtrDZ = 10;
int drvtrFB;
int drvtrLR;

int intakePTOvalue = false;



#endif