#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <vex.h>
#include <devices.hpp>

double sgn(double x);
double clamp(double x, double min, double max);
double clamp_abs(double x, double min, double max);

#define INTAKE_BIND     controller.ButtonR1
#define OUTTAKE_BIND    controller.ButtonR2
#define LB_RAISE_BIND   controller.ButtonL1
#define LB_LOWER_BIND   controller.ButtonL2
#define MOGO_BIND       controller.ButtonA

#endif // #ifndef DEFINITIONS_HPP