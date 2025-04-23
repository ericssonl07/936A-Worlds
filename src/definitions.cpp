#include <definitions.hpp>

// Utility functions
double sgn(double x) {
    return (x > 0) - (x < 0);
}

double clamp(double x, double min, double max) {
    return (x < min) ? min : (x > max) ? max : x;
}

double clamp_abs(double x, double min, double max) {
    return clamp(x, -max, max);
}

void move_motor(vex::motor m, int pct) {
    m.spin(vex::fwd, 128 * pct, vex::voltageUnits::mV);
}

// Chassis
bool g_toggle_base_pto = false;
int  g_right_power      = 0;
int  g_left_power       = 0;

// Intake
double get_intake_pos() {
    return intake_hook_rotation.position(vex::rotationUnits::deg);
}

int get_intake_stage() {
    return floor(get_intake_pos() / g_intake_period);
}

double get_intake_hook_pos() {
    return get_intake_pos() - get_intake_stage() * g_intake_period;
}

bool g_team_color        = 1; // 1 = red, 0 = blue
bool g_toggle_intake_pto = false; // false -> hooks connected, true -> hooks disconnected
int  g_intake_power      = 0;
bool g_lb_macro_mode   = false; // true -> macro mode, false -> manual mode

// Ladybrown
double get_lb_pos() {
    return lb_motor.position(vex::rotationUnits::deg);
}
int g_lb_power = 0;
int g_lb_target_arm_height = 0;

PID g_lb_pid(0.7, 0.0, 3.0, 0.0, 1.0, 100.0, -100.0, 0.0, 0.0, 1.0, 10000.0, 1.0, true); 

// Mogo
bool g_toggle_mogo = false;

bool g_toggle_hang_piston = false;