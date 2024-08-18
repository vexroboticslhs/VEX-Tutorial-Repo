#include "main.h"

using namespace pros;

void stop_motors() {
	left_dt.move_voltage(0);
    right_dt.move_voltage(0);
}

// time in milliseconds
void drive_time(int time) {
    int dir;
    if(time < 0){
        dir = -1;
        time *= -1;
    } else{
        dir = 1;
    }
    left_dt.move_voltage(12000 * dir);
    right_dt.move_voltage(12000 * dir);
    pros::delay(time);
    void stop_motors();
}

int sign(double num) {
  return (num > 0) ? 1 : (num < 0) ? -1 : 0;
}

// This is an example implementation of a lookup function for turning. 
// All constants are set to 0, you will need to tune them.
std::tuple<double, double, double> turn_lookup(double degrees) {
	// Tuples group multiple values of different types into a single object. 
	// In this case, we are using tuples to return a triple (kP, kI, kD)
	std::tuple<double, double, double> constants;
	
	// We tune for angle ranges rather than one specific angle.
	// The first angle range is from 0 to 15 degrees.
	if (fabs(degrees) <= 15) {
		get<0>(constants) = 0; 
		get<1>(constants) = 0; 
		get<2>(constants) = 0; 
	}
	else if (fabs(degrees) <= 30) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 40) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0; 
	} 
	else if (fabs(degrees) <= 50) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0; 
	}
	else if (fabs(degrees) <= 60) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 70) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 80) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 90){
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 105) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 120) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 135) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 150) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(degrees) <= 165) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else { // 180 degrees
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	
	return constants;
}

// This is an example implementation of a lookup function for straight drive. 
// All constants are set to 0, you will need to tune them.
std::tuple<double, double, double> drive_lookup(double displacement) {
	std::tuple<double, double, double> constants;

	if (fabs(displacement) <= 8) {
		get<0>(constants) = 0; 
		get<1>(constants) = 0; 
		get<2>(constants) = 0; 
	}
	else if (fabs(displacement) <= 15) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(displacement) <= 25) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0; 
	} 
	else if (fabs(displacement) <= 35) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0; 
	}
	else if (fabs(displacement) <= 45) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(displacement) <= 55) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else if (fabs(displacement) <= 65) {
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	else { 
		get<0>(constants) = 0;
		get<1>(constants) = 0;
		get<2>(constants) = 0;
	}
	
	return constants;
}

void turn(double angle) {
	// constants
	std::tuple<double, double, double> constants = turn_lookup(angle);
	double kP = get<0>(constants);
	double kI = get<1>(constants);
	double kD = get<2>(constants);
	
	// variables
	double error;
	double integral = 0;
	double derivative;
	
	double previous_error = angle;
	
	double error_threshold = 1.5; 
	double error_timer = 0; 
	double total_timer = 0; 
	
	double error_timeout = 150; 
	double max_timeout = 1200; 
	
	double integral_threshold = 1000;
	
	left_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
	right_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
	
	double target = inertial.get_rotation() + angle;
	
	// PID loop
	while (!(error_timer >= error_timeout) || !(total_timer >= max_timeout)) {
		error = target - inertial.get_rotation();
	
		if (sign(previous_error) != sign(error)) {
			integral = 0;
		}
		
		integral += error;
		integral = std::clamp(integral, -integral_threshold, integral_threshold);
		
		derivative = error - previous_error;
		previous_error = error;
		
		double output = error * kP + integral * kI + derivative * kD;
		// To turn, we spin the motors in opposite directions.
		// Try doing this with the robot manually to see the mechanics and physics.
		left_dt.move_voltage(output);
		right_dt.move_voltage(-output);
		
		if (error < error_threshold) {
			error_timer += 10;
		}
		total_timer += 10;
		
		delay(10);
	}
	stop_motors();
}


// Instead of an angle, now we have displacement.
void drive_dist(double displacement) {
	std::tuple<double, double, double> constants = drive_lookup(displacement);
	double kP = get<0>(constants);
	double kI = get<1>(constants);
	double kD = get<2>(constants);
	
	double error;
	double integral = 0;
	double derivative;
	
	double previous_error = displacement;
	
	double error_threshold = 1; 
	double error_timer = 0; 
	double total_timer = 0; 
	
	double error_timeout = 150; 
	double max_timeout = 2000; 
	
	double integral_threshold = 500;

	// This is the value we used for angle correction, it's up to you to change.
	double kTheta = 400; 
	double initial_angle = inertial.get_rotation();
	
	left_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
	right_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
	
	// From Sensors & Measurements section
	double target = total_displacement() + displacement;
	
	// PID loop
	while (!(error_timer >= error_timeout) || !(total_timer >= max_timeout)) {
		error = target - total_displacement();
		
		if (sign(previous_error) != sign(error)) {
			integral = 0;
		}
		
		integral += error;
		integral = std::clamp(integral, -integral_threshold, integral_threshold);
		
		derivative = error - previous_error;
		previous_error = error;
		
		
		/* 
		We calculate kTurn, the angle correction. We add kTurn to the left motor
		and subtract it from the right motor. This makes the robot turn slightly while
		moving forward/backward.
		*/
		double kTurn = kTheta * (initial_angle - inertial.get_rotation());
		double output = error * kP + integral * kI + derivative * kD;
		
		// Adjust the voltages accordingly
		left_dt.move_voltage(output + kTurn);
		right_dt.move_voltage(output - kTurn);
		
		if (error < error_threshold) {
			error_timer += 10;
		}
		total_timer += 10;
		
		delay(10);
	}
	stop_motors();
}

