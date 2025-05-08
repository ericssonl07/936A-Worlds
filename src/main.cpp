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

#define DEFAULT_FORWARD_PARAMS 1.0, 12.0, 3.5, 24.0, 12.0, 0.2, 0.9, 0.08, 5.0
#define DEFAULT_TURN_PARAMS 0.05, 12.0, 3.5, M_PI / 2, M_PI / 4, M_PI / 100, 8.0, 0.45, 35.0

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
double max_radial_accel = 100.0; // PURE PURSUIT SMOOTHNESS/EFFICIENCY TRADEOFF: TUNE THIS
double base_width = 11.25;
double left_offset = 1.34940945;
double back_offset = 2.29724409;
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

auto distance = [] (double x, double y, double x2, double y2) {
	return sqrt(pow(x2 - x, 2) + pow(y2 - y, 2));
};

#define MACROMODE(height) do { \
	base.lb_macro_mode = true; \
	base.lb_target_arm_height = height; \
} while (0);
#define STOREOFF do { \
	base.lb_macro_mode = false; \
} while (0);

void activaterightdoinkerafterdelay() { vex::this_thread::sleep_for(500); base.toggle_ring_doinker = true; printf("doinker activated\n"); }
void activateleftdoinkerafterdelay() { vex::this_thread::sleep_for(500); base.toggle_goal_doinker = true; printf("doinker activated\n"); }
void setloadmodeaftertworingsin() { if (base.intake_ring_fire_count == 2) {MACROMODE(base.lb_load_height);} }
// started 4:55, deployed in 0:33, arrives 5:68 (roughly)

int blue_ringrush() {
	base.set_pose(123.01, 100.24, 2.79);
	base.team_color = 0; // blue (0 = blue, 1 = red)
	double mp = 1.00; // p constant
	double md = 0.22; // d constant
	double mi = 3.0; // i constant
	base.lb_store_mode_override = true;
	base.intake_power = 100;
	vex::thread activatedoinkerafterdelaythread(activaterightdoinkerafterdelay);
	base.forward(distance(123.73, 104.31, 78, 118)-5, 1.0, 12.8, 24, 6, 0.2);
	Path back_into_mogo = {
		{
			{base.x(), base.y()},
			{87.9, 111.9},
			{96, 96}
		}, -1
	};
	vex::this_thread::sleep_for(100);
	base.intake_power = 0;
	base.follow_path(back_into_mogo, 1.0, 5.0);
	base.toggle_ring_doinker = false;
	base.toggle_mogo = true;
	base.intake_power = 100;
	base.turn_to(atan2(120 - base.y(), 100 - base.x()), 0.1, 11.0, 1.5);
	base.forward(distance(base.x(), base.y(), 96, 125) - 1, 1.0, 12.8, 0.5, 24, 6, 0.2, mp, md, mi);
	base.turn_to(atan2(144 - base.y(), 144 - base.x()), 0.1, 11, 1.5);
	vex::this_thread::sleep_for(200);
	base.forward_timer(2, 9.0, 2);
	base.forward(-5.5, 1.0, 12.8, 24, 6, 0.2, mp, md, mi);
	base.forward_timer(1.3, 8, 0.35);
	base.forward(-5.5, 1.0, 12.8, 24, 6, 0.2, mp, md, mi);
	base.forward_timer(1.3, 8, 0.35);
	base.corner_reset_forward(7.0);
	base.steer_timer(-6.0, -6.0, .8);
	base.forward(-2, 1.0, 12.8, 0.5, 24, 6, 0.2, mp, md, mi);
	
	base.turn_to(atan2(72 - base.y(), 96 - base.x()), 0.1, 11.0, 1.5); // base.turn_to(3*M_PI/2, 0.1, 11.0, 1.5, 1.0, 0.4);
	MACROMODE(base.lb_descore2_height);

	base.forward(distance(base.x(), base.y(), 96, 72) - 5, 1.0, 12.8, 24, 6, 0.2, mp, 0.3, mi);


	// base.turn_to(0, 0.1, 11.0, 1.5);
	// base.forward_timer(3, 8, 0.5);

	// base.forward(-8, .5, 12.8, 0.5, 24, 6, 0.2, mp, md, mi);
	// STOREOFF;
	// base.lb_power = 100;
	// vex::this_thread::sleep_for(500);
	// base.lb_power = 0;
	// base.lb_store_mode_override = false;
    return 0;
}

int red_ringrush() {
	printf("Running red ringrush\n");
	base.set_pose(144 - 123.01, 100.24, M_PI - 2.79);
    base.team_color = 1; // red (0 = blue, 1 = red)
    double mp = 1.00;    // p constant
    double md = 0.22;    // d constant
    double mi = 3.0;     // i constant
    base.lb_store_mode_override = true;
    base.intake_power = 100;
    vex::thread activatedoinkerafterdelaythread(activateleftdoinkerafterdelay);
    base.forward(
        distance(144 - 123.73, 104.31, 144 - 78, 118) - 5,
        1.0, 12.8, 0.5/*, 0.5, 0.15*/
    );
    Path back_into_mogo = {
        {
            { base.x(), base.y() },
            { 144 - 87.9, 111.9 },   // (56.1, 111.9)
            { 144 - 96,    96   }    // (48,    96)
        },
        -1
    };
    vex::this_thread::sleep_for(100);
    base.intake_power = 0;
    base.follow_path(back_into_mogo, 1.0, 5.0);
    base.toggle_goal_doinker = false;
    base.toggle_mogo = true;
    base.intake_power = 100;
    base.turn_to(
        atan2(120 - base.y(), (144 - 100) - base.x()),
        0.1, 11.0, 1.5/*, 1.0, 0.4*/
    );
    base.forward(
        distance(base.x(), base.y(), 144 - 96, 125) - 1,
        1.0, 12.8, 24, 6, 0.2,
        mp, md, mi
    );
    base.turn_to(
        atan2(144 - base.y(), (144 - 144) - base.x()),  // atan2(144 - y, -x)
        0.1, 11.0, 1.5/*, 1.0, 0.4*/
    );
    vex::this_thread::sleep_for(200);
    base.forward_timer(2, 9.0, 2);
    base.forward(-5.5, 1.0, 12.8, 24, 6, 0.2, mp, md, mi);
    base.forward_timer(1.3, 8, 0.35);
    base.forward(-5.5, 1.0, 12.8, 24, 6, 0.2, mp, md, mi);
    base.forward_timer(1.3, 8, 0.35);
    // base.corner_reset_forward(7.0);
    base.steer_timer(-6.0, -6.0, 0.8);
    base.forward(-2, 1.0, 12.8, 0.5, 0.5, 0.2, mp, md, mi);
    // base.turn_to(-M_PI/2, 0.1, 11.0, 1.5, 1.0, 0.4);
	
	base.turn_to(atan2(72 - base.y(), 144 - 120 - base.x()), 0.1, 11.0, 1.5, 1.0, 0.4);
    MACROMODE(base.lb_load_height);
    base.forward(
        distance(base.x(), base.y(), 144 - 120, 72),
        1.0, 12.8, 0.5, 0.5, 0.15,
        mp, 0.3, mi
    );
    base.turn_to(M_PI, 0.1, 11.0, 1.5, 1.0, 0.4);
    base.forward_timer(3, 8, 0.5);
    base.forward(-8, 0.5, 12.8, 24, 6, 0.2, mp, md, mi);
    STOREOFF;
    base.lb_power = 100;
    vex::this_thread::sleep_for(500);
    base.lb_power = 0;
    base.lb_store_mode_override = false;
    return 0;
}

int control();

enum autontype {
	redringrush,
	blueringrush
};
autontype type = redringrush;

// int autonomous() {
void autonomous() {
	if (type == redringrush) {
		printf("Running red\n");
		red_ringrush();
	} else {
		printf("Running blue\n");
		blue_ringrush();
	}
	// return 0;
}

void competitioncontrol() {
	highstakes_control(&base);
}

int testing_auton() {
	// #define DEFAULT_FORWARD_PARAMS 1.0, 12.0, 3.5, 24.0, 8.0, 0.2, 0.9, 0.08, 4.0
	// #define DEFAULT_TURN_PARAMS 0.05, 12.0, 3.5, M_PI / 2, M_PI / 4, M_PI / 100, 5.0, 0.25, 25.0
	base.set_pose(0, 0, 0);
	// FORWARD FUNCTION- THE LAST SET OF PARAMETERS ARE TESTED FOR MULTIPLE DISTANCES (±6, ±12, ±24, ±48, ±72) and work well
	// Testing time with 4 back-and-forths of 24 and -24 inches
	// base.forward(24, 1.0, 12.0, 3.5, 24.0, 8.0, 0.2, 0.6, 0.08, 4.0); // initial stable version: 8.6 seconds
	// base.forward(24, 1.0, 12.0, 3.5, 24.0, 12.0, 0.2, 0.6, 0.08, 5.0); // changing integral threshold from 8 to 12 and d from 4 to 5: 8.4 seconds
	// base.forward(24, 1.0, 12.0, 3.5, 24.0, 8.0, 0.2, 0.7, 0.08, 4.0); // changing kp from 0.6 to 0.7: 8 seconds
	// base.forward(24, 1.0, 12.0, 3.5, 24.0, 8.0, 0.2, 0.9, 0.08, 4.0); // changing kp from 0.7 to 0.9: 7.5 seconds, but more jerky

	// TURN FUNCTION
	// Testing time with 4 back-and-forths of 180 and -180 degrees (relative- from 0 to M_PI and back is one iteration)
	// base.turn_to(M_PI, 0.05, 12.0, 3.5, M_PI / 2, M_PI / 5, M_PI / 100, 5.0, 0.15, 25.0); // initial stable version: 6.44 seconds
	// base.turn_to(M_PI, 0.05, 12.0, 3.5, M_PI / 2, M_PI / 4, M_PI / 100, 5.0, 0.15, 25.0); // changing integral threshold from pi/5 to pi/4: 6.11 seconds
	// base.turn_to(M_PI, 0.05, 12.0, 3.5, M_PI / 2, M_PI / 4, M_PI / 100, 5.0, 0.25, 25.0); // changing ki from 0.15 to 0.25: 5.98 seconds

	base.forward(24, 1.0, 12.0, 3.5, 24.0, 12.0, 0.2, 0.9, 0.08, 5.0); // forward params
	base.turn_to(M_PI, 0.05, 12.0, 3.5, M_PI / 2, M_PI / 4, M_PI / 100, 8.0, 0.45, 35.0); // turn params 6.7 seconds
	return 0;
}

int main() {
	vex::task ladybrown_task(ladybrown_thread, &base);
	vex::task intake_helper_task(intake_helper_thread, &base);
	vex::task intake_task(intake_thread, &base);
	vex::task mogo_task(mogo_thread, &base);
	vex::task pneumatics_task(pneumatics_thread, &base);
	vex::competition competition;
	imu.calibrate();
	printf("imu calibrating... ");
	while (imu.isCalibrating()) {
		vex::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	printf("done\n");
	// bool buttonr = false, lastbuttonr = false;
	// while (!controller.ButtonLeft.pressing()) {
	// 	controller.Screen.clearScreen();
	// 	controller.Screen.setCursor(1,1);
	// 	controller.Screen.print(type == redringrush ? "redringrush\n" : "blueringrush\n");
	// 	printf(type == redringrush ? "redringrush\n" : "blueringrush\n");
	// 	buttonr = controller.ButtonRight.pressing();
	// 	if (buttonr && !lastbuttonr) {
	// 		if (type == redringrush) type = blueringrush;
	// 		else type = redringrush;
	// 	}
	// 	lastbuttonr = buttonr;
	// 	vex::this_thread::sleep_for(std::chrono::milliseconds(100));
	// }
	// competition.autonomous(autonomous);
	// competition.drivercontrol(competitioncontrol);

	vex::task testing_auton_task(testing_auton);
	while (!controller.ButtonUp.pressing()) {
		vex::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	testing_auton_task.stop();

	// vex::task auton(autonomous);
	// while (!controller.ButtonUp.pressing()) {
	// 	vex::this_thread::sleep_for(std::chrono::milliseconds(100));
	// }
	// auton.stop();
	
	// vex::task movement_task(control);
	vex::task control_task(highstakes_control, &base);
	while (true) {
		// printf("(%.5f, %.5f, %.5f)\n", base.x(), base.y(), base.rotation() / M_PI * 180);
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