#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

//Controllers
    extern pros::Controller Master;

//Motors
    //Drivetrain 
        extern pros::Motor FrontLeft;
        extern pros::Motor MidLeft;
        extern pros::Motor BackLeft;

        extern pros::Motor FrontRight;
        extern pros::Motor MidRight;
        extern pros::Motor BackRight;

        extern pros::MotorGroup LeftWheels;
        extern pros::MotorGroup RightWheels;

        extern pros::MotorGroup AllWheels;

    //Intake
        extern pros::Motor Preroller;
        extern pros::Motor Transport;

        extern pros::MotorGroup Intake;

    //Arm
        extern pros::Motor Arm;
    
    //Clamp
        extern pros::ADIDigitalOut Clamp;

    // Rotational Sensors
        extern pros::Rotation Rotational;
        extern pros::Rotation RotationalTurn;

    // Other Sensors
        extern pros::Optical colorSense;
        extern pros::Distance Distance;

    // Inertial Sensors/Kalman Filters
        extern pros::IMU Inertial1;
        extern KalmanFilter Kalman1;
        extern pros::IMU Inertial2;
        extern KalmanFilter Kalman2;

// Task Pointers

    extern pros::Task* controlLoop_task_ptr;
    extern pros::Task* coordinateUpdater_task_ptr;
    extern pros::Task* rotationalBinder_task_ptr ;
    extern pros::Task* autoSelector_task_ptr;

// Variables

    extern int num;
    extern bool clampOn;

    extern int autonnumber;
    extern bool globalAuton;
    extern int logoCount;

    extern int drvtrDZ;
    extern int drvtrFB;
    extern int drvtrLR;
    // upright = 1900, upleft = -1900

    extern bool toggleColorSensor;
    extern int colorDelay;
    extern bool ejectIsEjecting;

    extern bool presettingA;
    extern bool presettingX;

    extern Point endCoords;
    extern bool endReverse;
    extern bool endended;

// declared in initialize(), not init.cpp
    extern Point universalCurrentLocation;

    extern pros::screen_touch_status_s_t status;

#endif