#include <chassis.hpp>

Chassis::Chassis(MotorGroup* left, MotorGroup* right, vex::rotation* left_track, vex::rotation* back_track, vex::inertial* imu, vex::controller* controller,
                 double base_width, double left_offset, double back_offset, double wheel_radius, double tracking_radius, double external_ratio, double max_radial_acceleration)
    : left(left), right(right), left_track(left_track), back_track(back_track), imu(imu), controller(controller), base_width(base_width),
      left_offset(left_offset), back_offset(back_offset), wheel_radius(wheel_radius), tracking_radius(tracking_radius),
      external_ratio(external_ratio), max_radial_acceleration(max_radial_acceleration) {
    odometry = new Odometry(left_track, back_track, imu, left_offset, back_offset, tracking_radius);
}

void Chassis::steer(double left_voltage, double right_voltage) {
    double max_voltage = std::max(fabs(left_voltage), fabs(right_voltage));
    if (max_voltage > 12.0) {
        left_voltage *= 12.0 / max_voltage;
        right_voltage *= 12.0 / max_voltage;
    }
    left -> spin(left_voltage, vex::volt);
    right -> spin(right_voltage, vex::volt);
}

void Chassis::set_pose(double x, double y, double rotation) {
    odometry -> set_pose(x, y, rotation);
}

void Chassis::follow_path(Path path, double tolerance, double lookahead) {
    Pursuit pursuit(path, lookahead);
    while (!pursuit.terminated(x(), y())) {
        double progress = pursuit.progress();
        double voltage_limit = 4 / (1 + exp((progress - 0.75) * 20)) + 8.0; // \frac{4}{1+e^{20\left(x-0.75\right)}}+8- logistic function
        auto [steering, curvature] = pursuit.get_relative_steering(x(), y(), rotation(), base_width, voltage_limit);
        auto [left_velocity, right_velocity] = steering;
        
        // auto [target_x, target_y] = pursuit.get_target(x(), y());
        // printf("(%.5f, %.5f) -> (%.5f, %.5f): (%.5f, %.5f)\n\n\n", x(), y(), target_x, target_y, left_velocity, right_velocity); vexDelay(1);
        // printf("(%.5f, %.5f)\n", x(), y());

        #define RESTRICT_VELOCITY // Uncomment this line to restrict the velocity
        #ifdef RESTRICT_VELOCITY
        const double max_velocity0 = sqrt(fabs(curvature * max_radial_acceleration)); // v=√(ar)
        const double v_wheel_max = left -> max_wheel_velocity();
        const double left_tangential_as_pct_max = left_velocity / 12.0;
        const double left_tangential = v_wheel_max * left_tangential_as_pct_max;
        const double angular_speed = left_tangential / fabs(curvature - base_width * 0.5); // |omega|=v/r_left
        const double center_tangential = curvature * angular_speed; // v=|omega|*r_center
        double correction_ratio = max_velocity0 / fabs(center_tangential);
        correction_ratio = correction_ratio < 1.0 ? correction_ratio : 1.0;
        left_velocity *= correction_ratio;
        right_velocity *= correction_ratio;
        #endif

        steer(left_velocity, right_velocity);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    printf("Done\n");
    left -> stop();
    right -> stop();
}

void Chassis::turn_to(double angle, double tolerance, double maximum, double minimum, double activation_ratio, double integral_ratio) {
    double target = floor((rotation() - angle + M_PI) / (M_PI * 2)) * M_PI * 2 + angle;
    double error0 = target - rotation();
    turn(error0, tolerance, maximum, minimum, activation_ratio, integral_ratio);
}

void Chassis::turn(double angle, double tolerance, double maximum, double minimum, double activation_ratio, double integral_ratio) {
    double target = rotation() + angle;
    double error0 = angle;
    PID turn_controller(5.0, 0.3, 15.0, // adjust gains // 5 0.3 5
                        target, // target position
                        tolerance, // tolerance: allowed error
                        maximum, // max_value
                        minimum, // min_value
                        fabs(error0 * activation_ratio), // activation_threshold
                        fabs(error0 * integral_ratio), // integration_threshold
                        0.05, // derivative_threshold
                        50.0, // max_integral
                        0.999); // gamma
    while (!turn_controller.arrived()) {
        double pos = rotation();
        double output = turn_controller.calculate(pos);
        left -> spin(-output, vex::voltageUnits::volt);
        right -> spin(output, vex::voltageUnits::volt);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    left -> stop();
    right -> stop();
    printf("Done\n");
}

void Chassis::forward(double distance, double correction_strength, double tolerance, double maximum, double minimum, double activation_ratio, double integral_ratio) {
    double target_x = x() + distance * cos(rotation());
    double target_y = y() + distance * sin(rotation());
    double original_x = x();
    double original_y = y();
    auto dist = [this, original_x, original_y] () {
        double dx = x() - original_x;
        double dy = y() - original_y;
        return sqrt(dx * dx + dy * dy);
    };
    PID forward_controller(0.5, 0.0, 2.0, // adjust gains
                           distance, // target position
                           tolerance, // tolerance: allowed error
                           maximum, // max_value
                           minimum, // min_value
                           fabs(distance * activation_ratio), // activation_threshold
                           fabs(distance * integral_ratio), // integration_threshold
                           0.05, // derivative_threshold
                           50.0, // max_integral
                           0.999); // gamma
    while (!forward_controller.arrived()) {
        double pos = dist();
        double output = forward_controller.calculate(pos);
        double progress = dist() / distance;
        double clamp = sqrt(1.0 - pow(progress, 3.0));
        double correction = correction_strength * atan2(target_y - y(), target_x - x()) * clamp;
        const double m_c = 1.0;
        correction = correction < -m_c ? -m_c : (correction > m_c ? m_c : correction);
        printf("%.5f\n", rotation() / M_PI * 180.0);
        // printf("(%.5f, %.5f, %.5f)\n", x(), y(), rotation());
        left -> spin(output - correction, vex::voltageUnits::volt);
        right -> spin(output + correction, vex::voltageUnits::volt);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    left -> stop();
    right -> stop();
    printf("Done\n");
}

void Chassis::reset_position() {
    odometry -> set_pose(0.0, 0.0, 0.0);
}

void Chassis::stop() {
    left -> stop();
    right -> stop();
}

double Chassis::x() {
    return odometry -> x();
}

double Chassis::y() {
    return odometry -> y();
}

double Chassis::rotation() {
    return odometry -> rotation();
}

int control_by_wire(void* o) {
    Chassis* chassis = static_cast<Chassis*>(o);
    vex::controller& controller = *(chassis -> controller);
    vex::inertial& imu = *(chassis -> imu);
    MotorGroup& left = *(chassis -> left);
    MotorGroup& right = *(chassis -> right);
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

int default_control(void* o) {
    Chassis* chassis = static_cast<Chassis*>(o);
    vex::controller& controller = *(chassis -> controller);
    MotorGroup& left = *(chassis -> left);
    MotorGroup& right = *(chassis -> right);
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

// int control(void* o) {
//     Chassis* chassis = static_cast<Chassis*>(o);
//     vex::controller& controller = *(chassis -> controller);
//     MotorGroup& left = *(chassis -> left);
//     MotorGroup& right = *(chassis -> right);
//     vex::task controller_by_wire(control_by_wire, chassis);
//     vex::task default_controller(default_control, chassis);
//     controller_by_wire.suspend();
//     bool is_default_control = true;
//     bool last_button_a = false;
//     unsigned long long loop_iter = 0;
//     bool last_c[6] = {left[0].connected(), left[1].connected(), left[2].connected(), 
//                         right[0].connected(), right[1].connected(), right[2].connected()};
//     while (true) {
//         bool c[] = {left[0].connected(), left[1].connected(), left[2].connected(),
//                             right[0].connected(), right[1].connected(), right[2].connected()};
//         if (
//             (!c[0] && last_c[0]) ||
//             (!c[1] && last_c[1]) ||
//             (!c[2] && last_c[2]) ||
//             (!c[3] && last_c[3]) ||
//             (!c[4] && last_c[4]) ||
//             (!c[5] && last_c[5])
//         ) {
//             controller.rumble("-..");
//             if (is_default_control) {
//                 controller_by_wire.resume();
//                 default_controller.suspend();
//                 is_default_control = false;
//             }
//         }
//         last_c[0] = c[0]; last_c[1] = c[1]; last_c[2] = c[2];
//         last_c[3] = c[3]; last_c[4] = c[4]; last_c[5] = c[5];
//         if (loop_iter++ % 25 == 0) {
//             controller.Screen.clearScreen();
//             controller.Screen.setCursor(1, 1); controller.Screen.print(c[0] ? "C" : "X");
//             controller.Screen.setCursor(2, 1); controller.Screen.print(c[1] ? "C" : "X");
//             controller.Screen.setCursor(3, 1); controller.Screen.print(c[2] ? "C" : "X");
//             controller.Screen.setCursor(1, 2); controller.Screen.print(c[3] ? "C" : "X");
//             controller.Screen.setCursor(2, 2); controller.Screen.print(c[4] ? "C" : "X");
//             controller.Screen.setCursor(3, 2); controller.Screen.print(c[5] ? "C" : "X");
//             if (!is_default_control) {
//                 controller.Screen.setCursor(1, 7); controller.Screen.print("W");
//             }
//         }
//         bool button_a = controller.ButtonA.pressing();
//         if (button_a && !last_button_a) {
//             if (is_default_control) {
//                 controller_by_wire.resume();
//                 default_controller.suspend();
//             } else {
//                 controller_by_wire.suspend();
//                 default_controller.resume();
//             }
//             is_default_control = !is_default_control;
//         }
//         last_button_a = button_a;
//         vex::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
//     return 0;
// }