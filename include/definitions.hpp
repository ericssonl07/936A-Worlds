#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <vex.h>
#include <devices.hpp>

extern double sgn(double x);
extern double clamp(double x, double min, double max);
extern double clamp_abs(double x, double min, double max);
extern void move_motor(vex::motor m, int pct);

#define DRIVE_AXIS_FORWARD controller.Axis3.position()
#define DRIVE_AXIS_TURN    controller.Axis1.position()
#define INTAKE_BIND        controller.ButtonR1.pressing()
#define OUTTAKE_BIND       controller.ButtonR2.pressing()
#define LB_RAISE_BIND      controller.ButtonL1.pressing()
#define LB_LOWER_BIND      controller.ButtonL2.pressing()
#define MOGO_BIND          controller.ButtonX.pressing()
#define PTO_BIND           controller.ButtonA.pressing()

// chassis
extern bool g_toggle_base_pto;
extern int g_right_power;
extern int g_left_power;

// ladybrown

extern int g_lb_power;

#endif // DEFINITIONS_HPP