#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

//Controllers
    extern pros::Controller Master;

//Motors
    
    extern pros::Motor FrontLeft;
    extern pros::Motor MidLeft;
    extern pros::Motor BackLeft;
    extern pros::Motor_Group LeftWheels;

    extern pros::Motor FrontRight;
    extern pros::Motor MidRight;
    extern pros::Motor BackRight;
    extern pros::Motor_Group RightWheels;

    extern pros::Motor_Group AllWheels;

    
    extern pros::Motor InputMotor;
    extern pros::Motor Transport;
    extern pros::Motor_Group Intake;

    extern pros::Motor Arm;

/*
ABCDEFGH
12345678
Pnuematics
*/

    extern pros::ADIDigitalOut MobileGoalManipulator;
    extern pros::ADIDigitalOut InputPiston;
    extern pros::ADIDigitalOut ArmPiston;
    extern pros::ADIDigitalOut Grabber;
    extern pros::ADIDigitalOut Hang;


// Tasks

    extern pros::Task* coordinateUpdater_task_ptr;


// Sensors

    extern pros::Optical colorSense;
    extern pros::Distance Distance;

    extern pros::Rotation Rotational;
    extern pros::Rotation RotationalTurn;

    extern pros::IMU Inertial1;
    extern KalmanFilter Kalman1;
    extern pros::IMU Inertial2;
    extern KalmanFilter Kalman2;

// Variables
extern int autonnumber;
extern int logoCount;

extern int drvtrDZ;
extern int drvtrFB;
extern int drvtrLR;
// upright = 1900, upleft = -1900

extern bool toggleColorSensor;
extern int colorDelay;

extern bool redirectOn;
extern int redirectStartPoint;

extern bool presettingA;
extern bool presettingX;

// declared in initialize(), not init.cpp
extern Coordinate universalCurrentLocation;

#endif