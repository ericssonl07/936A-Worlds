#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <vex.h>
#include <deque>
#include <devices.hpp>
#include <pid.hpp>

// Utility functions
extern double sgn(double x); // Declare sgn as an extern function
extern double clamp(double x, double min, double max);
extern double clamp_abs(double x, double min, double max);
extern void   move_motor(vex::motor m, int pct);

// Controller bindings
#define DRIVE_AXIS_FORWARD controller.Axis3.position()
#define DRIVE_AXIS_TURN    controller.Axis1.position()
#define INTAKE_BIND        controller.ButtonR1.pressing()
#define OUTTAKE_BIND       controller.ButtonR2.pressing()
#define LB_RAISE_BIND      controller.ButtonL1.pressing()
#define LB_LOWER_BIND      controller.ButtonL2.pressing()
#define MOGO_BIND          controller.ButtonDown.pressing()
#define PTO_BIND           controller.ButtonB.pressing()
#define HANG_BIND          controller.ButtonA.pressing()

// Chassis
extern bool g_toggle_base_pto;
extern int  g_right_power;
extern int  g_left_power;

// Intake
extern int    g_intake_power;
extern bool   g_toggle_intake_pto;
const double  g_intake_period = 19.0 * (360.0 / 6.0); // 19 chain links per hook
extern double get_intake_pos();
extern int    get_intake_stage();
extern double get_intake_hook_pos();
extern bool   g_team_color; // 1 = red, 0 = blue

struct Ring {
    int  hooked_stage;
    bool color;          // 1 = red, 0 = blue
    bool initial_color;  // 1 = red, 0 = blue

    Ring() {
        hooked_stage = get_intake_stage();
        color        = g_team_color;
    }

    double get_pos() {
        return get_intake_pos() - hooked_stage * g_intake_period;
    }
};

extern std::deque<Ring> g_intake_ring_queue;

// Ladybrown
extern double get_lb_pos();
extern int g_lb_power;
extern bool g_lb_macro_mode;
extern int g_lb_target_arm_height;

extern PID g_lb_pid; 

// Mogo
extern bool g_toggle_mogo;

// 
extern bool g_toggle_hang_piston;

#endif // DEFINITIONS_HPP