#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	controller.clear();
	
	// Reset inertial
	inertial.reset();

	// This is exactly like holding the brake of a car
	// Motors hold their position when they stop moving
	left_dt.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_dt.set_brake_mode(E_MOTOR_BRAKE_HOLD);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 void opcontrol() {
	left_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	right_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// Set pistons here to false too

	while (true) {
		left_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
		right_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

		// controller defined in globals
		// .get_analog() returns the reading [-127, 127] of a controller
		double left = ((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127.0);
		double right = ((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127.0); 

		// Adjust sensitivity as you see fit
		// left = left * 0.88;
		// right = right * 0.76;

		// Scaling, makes sure that the powers do not exceed the limit
		double mag = fmax(1.0, fmax(fabs(left + right), fabs(left - right)));
		
		// motor.move_velocity() takes inputs from [-600, 600] for blue motors.
		// Red is [-100, 100] and green is [-200, 200]
		// A negative input spins the motor in the opposite direction than set in globals
		double left_power = ((left + right) / mag) * 600;
		double right_power = ((left - right) / mag) * 600;
		
		left_dt.move_velocity(left_power);
		right_dt.move_velocity(right_power);
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			turn(40);
		}

		// Delay is in milliseconds
		// Important because in a while loop, this is being repeated infinitely
		// We do not want the brain to crash
		pros::delay(10);
	}

}