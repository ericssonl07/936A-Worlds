/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ericssonlin                                               */
/*    Created:      2/23/2025, 9:44:40 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <vex.h>
#include <motor.hpp>
#include <motor_group.hpp>
#include <odometry.hpp>
#include <pid.hpp>
#include <chassis.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

enum Bases {
	HKTechChallenge,
	Tier3HangPrototype,
	WorldsChampionship
};

#define BASE_TYPE HKTechChallenge
#define TILE * 24.0

// All in inches
double wheel_radius = 3.125 / 2;
double tracking_wheel_radius = 2.7 / 2;
double external_ratio = 48.0 / 72.0;
double max_radial_accel = 10.0;
double base_width = 11.25;
double left_offset = 0;
double back_offset = 5.6;
#if BASE_TYPE == HKTechChallenge
Motor lm1(vex::PORT10, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor lm2(vex::PORT6, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor lm3(vex::PORT2, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm1(vex::PORT16, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm2(vex::PORT17, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm3(vex::PORT19, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
MotorGroup left(external_ratio, wheel_radius, &lm1, &lm2, &lm3);
MotorGroup right(external_ratio, wheel_radius, &rm1, &rm2, &rm3);
vex::rotation left_track(vex::PORT7, true);
vex::rotation back_track(vex::PORT5, false);
vex::inertial imu(vex::PORT1, vex::turnType::left);
Chassis base(&left, &right, &left_track, &back_track, &imu, base_width, left_offset, back_offset, wheel_radius, tracking_wheel_radius, external_ratio, max_radial_accel);
vex::controller controller;
vex::brain brain;
#elif BASE_TYPE == Tier3HangPrototype
Motor lm1(vex::PORT10, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor lm2(vex::PORT9, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor lm3(vex::PORT8, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor rm1(vex::PORT20, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm2(vex::PORT19, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor rm3(vex::PORT18, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
MotorGroup left(external_ratio, wheel_radius, &lm1, &lm2, &lm3);
MotorGroup right(external_ratio, wheel_radius, &rm1, &rm2, &rm3);
vex::rotation left_track(vex::PORT7, true);
vex::rotation back_track(vex::PORT5, false);
vex::inertial imu(vex::PORT11, vex::turnType::left);
Chassis base(&left, &right, &left_track, &back_track, &imu, base_width, left_offset, back_offset, wheel_radius, tracking_wheel_radius, external_ratio, max_radial_accel);
vex::controller controller;
vex::brain brain;
#else
vex::brain brain;
vex::controller controller;
Motor lm1(vex::PORT13, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor lm2(vex::PORT12, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor lm3(vex::PORT11, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm1(vex::PORT3, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm2(vex::PORT2, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm3(vex::PORT1, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
MotorGroup left(external_ratio, wheel_radius, &lm1, &lm2, &lm3);
MotorGroup right(external_ratio, wheel_radius, &rm1, &rm2, &rm3);
vex::rotation left_track(vex::PORT7, true);
vex::rotation back_track(vex::PORT5, false);
vex::inertial imu(vex::PORT11, vex::turnType::left);
Chassis base(&left, &right, &left_track, &back_track, &imu, base_width, left_offset, back_offset, wheel_radius, tracking_wheel_radius, external_ratio, max_radial_accel);
#endif

// int autonomous() {
// 	vexDelay(1000);
// 	// base.forward(3 TILE, 1.5, 0.5, 12.8, 0.02, 0.3, 0.2); // 1.0 worked well
// 	// base.turn(M_PI / 2, 0.05, 12.0, 0.02, 0.3, 0.2);
// 	// Path path({{0, 0}, {1 TILE, 0.25 TILE}, {2 TILE, 0}, {3 TILE, -0.25 TILE}, {4 TILE, 0}}, 100);
// 	// base.follow_path(path, 0.5, 12.0);
// 	return 0;
// }

int control();
int control_by_wire();
int default_control();

int main() {
	imu.calibrate();
	vexDelay(3000);
	control();
}

int control() {
	vex::task controller_by_wire(control_by_wire);
	vex::task default_controller(default_control);
	controller_by_wire.suspend();
	bool is_default_control = true;
	bool last_button_a = false;
	unsigned long long loop_iter = 0;
	bool last_c[6] = {lm1.connected(), lm2.connected(), lm3.connected(), 
						rm1.connected(), rm2.connected(), rm3.connected()};
	while (true) {
		bool c[] = {lm1.connected(), lm2.connected(), lm3.connected(),
							rm1.connected(), rm2.connected(), rm3.connected()};
		if (
			(!c[0] && last_c[0]) ||
			(!c[1] && last_c[1]) ||
			(!c[2] && last_c[2]) ||
			(!c[3] && last_c[3]) ||
			(!c[4] && last_c[4]) ||
			(!c[5] && last_c[5])
		) {
			controller.rumble("-..");
			if (is_default_control) {
				controller_by_wire.resume();
				default_controller.suspend();
				is_default_control = false;
			}
		}
		last_c[0] = c[0]; last_c[1] = c[1]; last_c[2] = c[2];
		last_c[3] = c[3]; last_c[4] = c[4]; last_c[5] = c[5];
		if (loop_iter++ % 25 == 0) {
			controller.Screen.clearScreen();
			controller.Screen.setCursor(1, 1); controller.Screen.print(c[0] ? "C" : "X");
			controller.Screen.setCursor(2, 1); controller.Screen.print(c[1] ? "C" : "X");
			controller.Screen.setCursor(3, 1); controller.Screen.print(c[2] ? "C" : "X");
			controller.Screen.setCursor(1, 2); controller.Screen.print(c[3] ? "C" : "X");
			controller.Screen.setCursor(2, 2); controller.Screen.print(c[4] ? "C" : "X");
			controller.Screen.setCursor(3, 2); controller.Screen.print(c[5] ? "C" : "X");
			if (!is_default_control) {
				controller.Screen.setCursor(1, 7); controller.Screen.print("W");
			}
		}
		bool button_a = controller.ButtonA.pressing();
		if (button_a && !last_button_a) {
			if (is_default_control) {
				controller_by_wire.resume();
				default_controller.suspend();
			} else {
				controller_by_wire.suspend();
				default_controller.resume();
			}
			is_default_control = !is_default_control;
		}
		last_button_a = button_a;
		vex::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return 0;
}

int control_by_wire() {
	// Turn control parameters
	double kp = 0.5;    // Proportional gain
    double ki = 0.01;   // Integral gain
    double kd = 2.0;    // Derivative gain // 1.2, 1.5
	const double maxfloat = 3.4e38;
    PID heading_pid(kp, ki, kd,
				    0.0,  			// Initial target is set to the initial imu rotation
                    5.0,            // Tolerance (never terminates)
                    12.0, 0.0,     	// Max/min output (volts)
                    maxfloat,       // Activation threshold (always active)
                    maxfloat,       // Integration threshold
                    0.0,            // Derivative threshold (never terminates)
                    50.0,           // Max integral
                    0.99,           // Gamma (integral decay)
                    true);          // Deactivate integral on error sign change

    // Sensitivity and deadband parameters
    const double linear_scale = 12.0 / 100.0;	// Maximum linear speed is 12V
	
	// To turn at the same rate, set turn_rate_scale_drift = -0.035
	const double turn_rate_scale_fast = -0.035;	// Fast turn rate- turning in place
	// If you want to have better drifting, set this constant- (-0.01) seems to work alright
    const double turn_rate_scale_drift = -0.035; // Negative due to intuitive control convention (positive is clockwise, and not counterclockwise as in mathematical convention)
    const int deadband = 1;              	// Ignore inputs below 1%

    // Initialize target heading to the robot's current heading
    double target_heading = imu.rotation();

    while (true) {
		// Hack: when the controller toggles control by wire, the target heading is set to the current heading
		// This relies on "sticky" button behavior
		if (controller.ButtonA.pressing()) {
			target_heading = imu.rotation();
		}
        // Read controller inputs
		double linear = controller.Axis3.position() * linear_scale;  	// -100 to 100
		double angular = controller.Axis1.position();
		if (std::abs(controller.Axis3.position()) < deadband) {
			angular *= turn_rate_scale_fast; // -100 to 100
		} else {
			angular *= turn_rate_scale_drift; // -100 to 100
		}

        // Apply deadband
        if (std::abs(controller.Axis3.position()) < deadband) linear = 0.0;
        if (std::abs(controller.Axis1.position()) < deadband) angular = 0.0;

        double omega_des = angular; // Desired turn rate in deg/s

        // Update target heading by integrating the turn rate
        target_heading += omega_des;

        // Get current heading from inertial sensor
        double current_heading = imu.rotation();

        // Compute PID correction
        heading_pid.target(target_heading);
        double pid_output = heading_pid.calculate(current_heading); // Output in volts
		// Add an anticipative term
		pid_output += omega_des; // Scale as needed

        double left_power = linear - pid_output;
        double right_power = linear + pid_output;

        // Set motor powers
        left.spin(left_power, vex::volt);
        right.spin(right_power, vex::volt);
        vex::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

int default_control() {
	while (true) {
		double linear = controller.Axis3.position();
		double angular = controller.Axis1.position();
		double left_power = (linear + angular) * 0.12;
		double right_power = (linear - angular) * 0.12;
		left.spin(left_power, vex::volt);
		right.spin(right_power, vex::volt);
		vex::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return 0;
}