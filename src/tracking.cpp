#include "main.h"

using namespace pros;

double dt_rotation() {
	// dt.get_position_all()
	// returns a vector of the encoder measurements of all motors in MotorGroup
    std::vector<double> left_pos = left_dt.get_position_all();
    std::vector<double> right_pos = right_dt.get_position_all();

    double left_avg = (left_pos[0] + left_pos[1] + left_pos[2]) / 3;
    double right_avg = (right_pos[0] + right_pos[1] + right_pos[2]) / 3;

    return (left_avg + right_avg) / 2;
}

double total_displacement() {
	double delta = dt_rotation();
	
	// Multiply by gear ratio of drive train first
	delta *= gear_ratio;
	
	// The displacement of the robot after 1 rotation is equivalent to the circumference
	// of the robot's wheels. We can find this using the following equation:
	// Circumference = pi * wheel_diameter
	// Thus, we multiply the circumference.
	delta *= 3.1415926 * wheel_diameter;
	
	return delta;
}