#include "main.h"

using namespace pros;

// Depends on your robot
const double gear_ratio = 0.75;
const double wheel_diameter = 3.25;

// Motors
/*
Defining a motor:
	Motor motor_name(port, gearset, encoder_units);
	
	param port
		The V5 port number from 1 to 21, or from -21 to -1 for reversed motors. 
	
	param gearset = v5::MotorGears::green (default value if parameter is left blank)
		Optional parameter for the gearset for the motor.
		Use v5::MotorGears::[color]
	
	param encoderunits = v5::MotorUnits::degrees
		Optional parameter for the encoder units of the motor.
		Usually use degrees or rotations. This tutorial will use rotations.
*/
// Please note that these are subject to change depending on the robot's configuration.
Motor fl(-15, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor ml(-16, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor bl(-19, v5::MotorGears::blue, v5::MotorUnits::rotations);

Motor fr(11, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor mr(12, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor br(13, v5::MotorGears::blue, v5::MotorUnits::rotations);

// Motor intake(7, v5::MotorGears::green, v5::MotorUnits::rotations);

// Motor Groups
// Unfortunately, MotorGroup does not directly accept motor instances, only port numbers. 

MotorGroup left_dt({fl.get_port(), ml.get_port(), bl.get_port()}, v5::MotorGears::blue, v5::MotorUnits::rotations);
MotorGroup right_dt({fr.get_port(), mr.get_port(), br.get_port()}, v5::MotorGears::blue, v5::MotorUnits::rotations);

// Inertial
IMU inertial(7);

// Rotation Sensors
// Rotation left_rot(9); 
// Rotation right_rot(10);

// Pistons (Ports A-H)
adi::DigitalOut piston('A');

// Controller
Controller controller(E_CONTROLLER_MASTER);