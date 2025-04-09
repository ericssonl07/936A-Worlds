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

// enum Bases {
// 	HKTechChallenge,
// 	Tier3HangPrototype,
// 	WorldsChampionship
// };
// const constexpr static int HKTechChallenge_BASE = 0;
// const constexpr static int Tier3HangPrototype_BASE = 1;
// const constexpr static int WorldsChampionship_BASE = 2;

#define BASE_TYPE 1
#define TILE * 24.0

// All in inches
double wheel_radius = 3.125 / 2;
double tracking_wheel_radius = 2.7 / 2;
double external_ratio = 48.0 / 72.0;
double lb_ratio = 12.0 / 36.0;
double intake_ratio = 1.0;
double max_radial_accel = 100.0;
double base_width = 11.25;
double left_offset = 0;
double back_offset = 5.6;
vex::brain brain;
vex::controller controller;
#if BASE_TYPE == 0
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
Chassis base(&left, &right, &left_track, &back_track, &imu, &controller, base_width, left_offset, back_offset, wheel_radius, tracking_wheel_radius, external_ratio, max_radial_accel);
#elif BASE_TYPE == 1
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
Chassis base(&left, &right, &left_track, &back_track, &imu, &controller, base_width, left_offset, back_offset, wheel_radius, tracking_wheel_radius, external_ratio, max_radial_accel);
#else // BASE_TYPE == 2
Motor lm1(vex::PORT13, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor lm2(vex::PORT12, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor lm3(vex::PORT11, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm1(vex::PORT3, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm2(vex::PORT2, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius);
Motor rm3(vex::PORT1, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius);
Motor ladybrown(vex::PORT17, vex::gearSetting::ratio18_1, false, lb_ratio, 1.0);
Motor intake(vex::PORT19, vex::gearSetting::ratio6_1, true, intake_ratio, 1.0);
MotorGroup left(external_ratio, wheel_radius, &lm1, &lm2, &lm3);
MotorGroup right(external_ratio, wheel_radius, &rm1, &rm2, &rm3);
vex::rotation left_track(vex::PORT7, true); // TODO: install tracking wheels and adjust ports
vex::rotation back_track(vex::PORT5, false); // TODO: install tracking wheels and adjust ports
vex::rotation intake_track(vex::PORT20, false);
vex::inertial imu(vex::PORT11, vex::turnType::left);
vex::optical mogo_color(vex::PORT15);
vex::pneumatics mogo_piston(brain.ThreeWirePort.A);
vex::pneumatics intake_pto(brain.ThreeWirePort.B);
vex::pneumatics base_pto(brain.ThreeWirePort.C);
Chassis base(&left, &right, &left_track, &back_track, &imu, &controller, base_width, left_offset, back_offset, wheel_radius, tracking_wheel_radius, external_ratio, max_radial_accel);
#endif

int autonomous() {
	// vexDelay(1000);
	// base.forward(3 TILE, 1.5, 0.5, 12.8, 0.02, 0.3, 0.2); // 1.0 worked well
	// base.turn(M_PI / 2, 0.05, 12.0, 0.02, 0.3, 0.2);
	// Path path({{0, 0}, {1 TILE, 0.25 TILE}, {2 TILE, 0}, {3 TILE, -0.25 TILE}, {4 TILE, 0}}, 100);
	Path path({{0, 0}, {-1 TILE, -0.25 TILE}, {-2 TILE, 0}, {-3 TILE, 0.25 TILE}, {-4 TILE, 0}}, 100);
	base.follow_path(path, 2.5, 9.0);
	return 0;
}

int control();

int main() {
	imu.calibrate();
	vexDelay(3000);
	vex::task auton(autonomous);
	while (true) {
		if (controller.ButtonA.pressing()) {
			auton.stop();
			control();
		}
	}
}

int control() {
	vex::task controller_by_wire(control_by_wire, &base);
	vex::task default_controller(default_control, &base);
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

#if BASE_TYPE == 2 // Subsystem control for the Worlds Championship base
int subsystems_control() {
	while (true) {
		if (controller.ButtonR1.pressing()) {
			intake.spin(12.0, vex::voltageUnits::volt);
		} else if (controller.ButtonR2.pressing()) {
			intake.spin(-12.0, vex::voltageUnits::volt);
		} else {
			intake.stop();
		}
	}
}
#endif