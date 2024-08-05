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

double kP = 200;
double kI = 0.5;
double kD = 250;

void turn(double angle) {
	double error;
	double integral = 0;
	double derivative;
	
	double previous_error = angle;
	
	double error_threshold = 1.5; 
	double error_timer = 0; 
	double total_timer = 0; 
	
	double error_timeout = 150; 
	double max_timeout = 1200; 
	
	left_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    right_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
  
    double target = inertial.get_rotation() + angle;
  
    while (!(error_timer >= error_timeout) || !(total_timer >= max_timeout)) {
		error = target - inertial.get_rotation();
		
		integral += error;
		
		derivative = error - previous_error;
		previous_error = error;
		
		double output = error * kP + integral * kI + derivative * kD;
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