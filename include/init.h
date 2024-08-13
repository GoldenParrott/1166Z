#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

//Controllers
    extern pros::Controller Master;
    extern pros::Controller Partner;

//Motors

    extern pros::Motor UpLeft;
    extern pros::Motor UpRight;
    extern pros::Motor_Group IntakePTO;
    
    extern pros::Motor FrontLeft;
    extern pros::Motor BackLeft;
    extern pros::Motor_Group LeftWheels;
    extern pros::Motor_Group AllLeftWheels;

    extern pros::Motor FrontRight;
    extern pros::Motor BackRight;
    extern pros::Motor_Group RightWheels;
    extern pros::Motor_Group AllRightWheels;

    extern pros::Motor_Group AllWheels;
    extern pros::Motor_Group AllAllWheels;

    extern pros::Motor InputMotor;
    extern pros::Motor Transport;
    extern pros::Motor_Group Intake;

    extern pros::Motor ArmLeft;
    extern pros::Motor ArmRight;
    extern pros::Motor_Group Arm;

/*
ABCDEFGH
12345678
Pnuematics
*/

    extern pros::ADIDigitalOut MobileGoalManipulator;
    extern pros::ADIDigitalOut IntakePTOPiston;
    extern pros::ADIDigitalOut InputPiston;
    extern pros::ADIDigitalOut Eject;
    extern pros::ADIDigitalOut GrabPiston;


// Tasks

    extern pros::Task *colorSensorOn_task_ptr;


// Sensors

    extern pros::Optical colorSense;
    extern pros::IMU Inertial;

// Variables
extern int autonnumber;
extern int logoCount;

extern int drvtrDZ;
extern int drvtrFB;
extern int drvtrLR;
extern double armGoal;
extern double armmax;
extern double armPosition;
// upright = 1900, upleft = -1900

extern bool armCalibrated;
extern bool intakePTOvalue;
extern bool armMoving;
extern bool armMovementComplete;
extern bool toggleColorSensor;
extern int colorDelay;

extern bool presettingA;
extern bool presettingX;

#endif