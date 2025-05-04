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
#include <highstakes.hpp>

// enum Bases {
// 	HKTechChallenge,
// 	Tier3HangPrototype,
// 	WorldsChampionship
// };
// const constexpr static int HKTechChallenge_BASE = 0;
// const constexpr static int Tier3HangPrototype_BASE = 1;
// const constexpr static int WorldsChampionship_BASE = 2;

#define BASE_TYPE 2
#define TILE * 24.0

// All in inches
double wheel_radius = 3.125 / 2;
double tracking_wheel_radius = 2.75 / 2;
double external_ratio = 48.0 / 72.0;
double lb_ratio = 12.0 / 36.0;
double intake_ratio = 1.0;
double max_radial_accel = 100.0;
double base_width = 11.25;
double left_offset = 1.34940945; // 0 (wtf??)
double back_offset = 2.29724409; // 5.6 (wtf??)
// 59.675mm
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
// Worlds Championship
Motor lm1(vex::PORT18, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius); // correct
Motor lm2(vex::PORT20, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius); // correct
Motor lm3(vex::PORT17, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius); // correct
Motor rm1(vex::PORT2, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius); // correct
Motor rm2(vex::PORT4, vex::gearSetting::ratio6_1, false, external_ratio, wheel_radius); // correct
Motor rm3(vex::PORT3, vex::gearSetting::ratio6_1, true, external_ratio, wheel_radius); // correct
vex::motor lb1(vex::PORT19, vex::gearSetting::ratio18_1, false); // not wired
vex::motor lb2(vex::PORT16, vex::gearSetting::ratio18_1, true); // not wired
vex::motor intake(vex::PORT11, vex::gearSetting::ratio6_1, true); // correct
vex::inertial imu(vex::PORT1, vex::turnType::left); // correct
MotorGroup left(external_ratio, wheel_radius, &lm1, &lm2, &lm3);
MotorGroup right(external_ratio, wheel_radius, &rm1, &rm2, &rm3);
vex::rotation left_track(vex::PORT6, false); // not wired
vex::rotation back_track(vex::PORT7, true); // not wired
vex::optical intake_hook_color(vex::PORT12); // correct
vex::optical mogo_color(vex::PORT13); // correct
vex::optical first_stage_color(vex::PORT9); // correct
vex::distance lb_distance(vex::PORT14);
vex::rotation intake_rotation(vex::PORT5, false); // correct
vex::pneumatics mogo_piston(brain.ThreeWirePort.D); // not wired
vex::pneumatics intake_pto(brain.ThreeWirePort.F); // not wired
vex::pneumatics hang_piston(brain.ThreeWirePort.B); // not wired
vex::pneumatics base_pto(brain.ThreeWirePort.C); // not wired
vex::pneumatics intake_lift(brain.ThreeWirePort.E);
vex::pneumatics ring_doinker(brain.ThreeWirePort.H); // wired
vex::pneumatics goal_doinker(brain.ThreeWirePort.G); // wired
// Chassis base(&left, &right, &left_track, &back_track, &imu, &controller, base_width, left_offset, back_offset, wheel_radius, tracking_wheel_radius, external_ratio, max_radial_accel);
HighStakesChassis base(&left, &right, &left_track, &back_track, &imu, &controller,
	&intake, &lb1, &lb2, &base_pto, &intake_pto, &mogo_piston, &hang_piston,
	&ring_doinker, &goal_doinker, &intake_lift, &intake_hook_color, &mogo_color,
	&first_stage_color, &lb_distance, &intake_rotation,
	base_width, left_offset, back_offset, wheel_radius,
	tracking_wheel_radius, external_ratio, max_radial_accel);
#endif

int autonomous() {
	Path p1 = {
		{
			{24, 24},
			{48, 48},
			{72, 72},
			{96, 96},
			{72, 120},
			{24, 96},
			{24, 24}
		}, -1
	};
	base.follow_path(p1, 1.0, 5.0); // tolerance 1 inch, lookahead 5 inches
	vex::this_thread::sleep_for(std::chrono::milliseconds(500));
	base.turn_to(M_PI / 4, 0.1, 12.0, 0.02, 1.0, 0.4);
	vex::this_thread::sleep_for(std::chrono::milliseconds(500));
	base.steer_timer(-6, -7, 2.0);
	vex::this_thread::sleep_for(std::chrono::milliseconds(500));
	base.corner_reset(7.0);
	vex::this_thread::sleep_for(std::chrono::milliseconds(500));
	Path p2 = {
		{
			{base.x(), base.y()},
			{36, 36},
			{72, 24}
		}, -1
	};
	base.follow_path(p2, 1.0, 5.0); // tolerance 1 inch, lookahead 5 inches
	vex::this_thread::sleep_for(std::chrono::milliseconds(500));
	base.turn_to(M_PI, 0.1, 12.0, 0.02, 0.25, 0.1);
	vex::this_thread::sleep_for(std::chrono::milliseconds(500));
	base.forward(48, 1.0, 12.0, 0.5, 0.5, 0.15);
    return 0;
}

int control();

int main() {
	vex::task ladybrown_task(ladybrown_thread, &base);
	vex::task intake_helper_task(intake_helper_thread, &base);
	vex::task intake_task(intake_thread, &base);
	vex::task mogo_task(mogo_thread, &base);
	vex::task pneumatics_task(pneumatics_thread, &base);
	vex::task highstakes_control_task(highstakes_control, &base);
	imu.calibrate();
	vexDelay(3000);
	base.set_pose(0, 0, 0);

	base.lb_macro_mode = true;
	base.lb_target_arm_height = base.lb_load_height; // Target low height (adjust as needed)
	base.intake_power = 100;

	base.forward(48, 1.0, 12.8, 0.5, 0.5, 0.15);
	// vex::task auton(autonomous);
	// while (!controller.ButtonA.pressing()) {
	// 	printf("(%.5f, %.5f)\n", base.x(), base.y());
	// 	vex::this_thread::sleep_for(std::chrono::milliseconds(100));
	// }
	// auton.stop();



	while (true) {
		if (controller.ButtonA.pressing()) {
			base.corner_reset(7.0);
		}
		// printf("(%.5f, %.5f)\n", base.x(), base.y());
		vex::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

int control() {
	enum ControlMode {
		Wire,
		Arcade
		// Curvature,
		// Tank
	};
	ControlMode current_mode = Arcade;
	vex::task wire_controller(wire_control, &base);
	vex::task arcade_controller(arcade_control, &base);
	// vex::task curvature_controller(curvature_control, &base);
	// vex::task tank_controller(tank_control, &base);
	wire_controller.suspend();
	// curvature_controller.suspend();
	// tank_controller.suspend();
	auto switch_to = [&] (ControlMode mode) {
		if (current_mode == mode) {
			return;
		}
		current_mode = mode;
		switch (mode) {
			case Wire:
				wire_controller.resume(); arcade_controller.suspend(); /*curvature_controller.suspend(); tank_controller.suspend();*/
				break;
			case Arcade:
				wire_controller.suspend(); arcade_controller.resume(); /*curvature_controller.suspend(); tank_controller.suspend();*/
				break;
			// case Curvature:
			// 	wire_controller.suspend(); arcade_controller.suspend(); curvature_controller.resume(); tank_controller.suspend();
			// 	break;
			// case Tank:
			// 	wire_controller.suspend(); arcade_controller.suspend(); curvature_controller.suspend(); tank_controller.resume();
			// 	break;
		}
	};
	auto next_mode = [&] () {
		switch (current_mode) {
			case Wire:
				return Arcade;
			case Arcade:
				return Wire;
			//	 	return Curvature;
			// case Curvature:
			// 	return Tank;
			// case Tank:
			// 	return Wire;
		}
	};
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
			switch_to(Wire);
		}
		if (
			(c[0] && !last_c[0]) ||
			(c[1] && !last_c[1]) ||
			(c[2] && !last_c[2]) ||
			(c[3] && !last_c[3]) ||
			(c[4] && !last_c[4]) ||
			(c[5] && !last_c[5])
		) {
			controller.rumble("-.-.");
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
			switch (current_mode) {
				case Wire:
					controller.Screen.setCursor(1, 7); controller.Screen.print("Wire");
					break;
				case Arcade:
					controller.Screen.setCursor(1, 7); controller.Screen.print("Arcade");
					break;
				// case Curvature:
				// 	controller.Screen.setCursor(1, 7); controller.Screen.print("Curvature");
				// 	break;
				// case Tank:
				// 	controller.Screen.setCursor(1, 7); controller.Screen.print("Tank");
				// 	break;
			}
		}
		bool button_a = controller.ButtonA.pressing();
		if (button_a && !last_button_a) {
			switch_to(next_mode());
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
			intake.spin(vex::fwd, 12.8, vex::voltageUnits::volt);
		} else if (controller.ButtonR2.pressing()) {
			intake.spin(vex::fwd, -12.8, vex::voltageUnits::volt);
		} else {
			intake.stop();
		}
	}
}
#endif