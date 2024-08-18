#pragma once
#include "main.h"

// Because of this namespace, we do not need to explicitly write "pros::" before
// every PROS variable/function
using namespace pros;

// Drive train gear ratio
// const keyword indicates that the variable declared cannot change
extern const double gear_ratio; 
extern const double wheel_diameter;

// 6 motor drive (front/middle/back, left/right)
extern Motor fl;
extern Motor fr;
extern Motor ml;
extern Motor mr;
extern Motor bl;
extern Motor br;

// Intake
// extern Motor intake; 

// In PROS, one important function is grouping motors together. 
// Especially in differential drive trains, the left and right sides operate seperately.
// This allows functions to be called on the whole motor group.
extern MotorGroup left_dt;
extern MotorGroup right_dt;

// Inertial sensor
extern IMU inertial; 

// Rotation sensors
// extern Rotation left_rot; 
// extern Rotation right_rot;

// Example piston
extern adi::DigitalOut piston;

// Controller
extern Controller controller; 