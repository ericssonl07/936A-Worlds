#include <definitions.hpp>

double sgn(double x) {
    return 0.0;
}

double clamp(double x, double min, double max) {
    return 0.0;
}

double clamp_abs(double x, double min, double max) {
    return 0.0;
}

void move_motor(vex::motor m, int pct) {
	m.spin(vex::fwd, 128 * pct, vex::voltageUnits::mV);	
}

bool g_toggle_base_pto = false;

int g_right_power = 0;
int g_left_power = 0;

int g_lb_power = 0;