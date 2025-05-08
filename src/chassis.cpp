#include <chassis.hpp>

Chassis::Chassis(MotorGroup* left, MotorGroup* right, vex::rotation* forward_track, vex::rotation* side_track, vex::inertial* imu, vex::controller* controller,
                 double base_width, double left_offset, double back_offset, double wheel_radius, double tracking_radius, double external_ratio, double max_radial_acceleration)
    : left(left), right(right), forward_track(forward_track), side_track(side_track), imu(imu), controller(controller), base_width(base_width),
      forward_offset(left_offset), side_offset(back_offset), wheel_radius(wheel_radius), tracking_radius(tracking_radius),
      external_ratio(external_ratio), max_radial_acceleration(max_radial_acceleration) {
    odometry = new Odometry(forward_track, side_track, imu, left_offset, back_offset, tracking_radius);
}

void Chassis::steer(double left_voltage, double right_voltage) {
    double max_voltage = std::max(fabs(left_voltage), fabs(right_voltage));
    if (max_voltage > 12.8) {
        left_voltage *= 12.8 / max_voltage;
        right_voltage *= 12.8 / max_voltage;
    }
    left -> spin(left_voltage, vex::volt);
    right -> spin(right_voltage, vex::volt);
}

void Chassis::set_pose(double x, double y, double rotation) {
    odometry -> set_pose(x, y, rotation);
}

void Chassis::follow_path(Path path, double tolerance, double lookahead) {
    printf("Called follow_path... ");
    Pursuit pursuit(path, lookahead);
    const double min_voltage = 1.5; // PURE PURSUIT SMOOTHNESS/EFFICIENCY TRADEOFF: TUNE THIS
    // auto initial_target = pursuit.get_target(x(), y());
    // double angle = atan2(initial_target.y - y(), initial_target.x - x());
    // turn_to(angle, 0.5, 6.0, 1.5, M_PI / angle * 0.25, M_PI / angle * 0.1);
    while (!pursuit.terminated(x(), y())) {
        double progress = pursuit.progress();
        double voltage_limit = 2 / (1 + exp((progress - 0.9) * 20)) + 10;
        auto [steering, curvature] = pursuit.get_relative_steering(x(), y(), rotation(), base_width, voltage_limit);
        auto [left_velocity, right_velocity] = steering;
        #define RESTRICT_VELOCITY // Uncomment this line to restrict the velocity
        #ifdef RESTRICT_VELOCITY
        const double max_velocity0 = sqrt(fabs(curvature * max_radial_acceleration)); // v=√(ar)
        const double v_wheel_max = left -> max_wheel_velocity();
        const double left_tangential_as_pct_max = left_velocity / 12.0;
        const double left_tangential = v_wheel_max * left_tangential_as_pct_max;
        const double angular_speed = left_tangential / fabs(curvature - base_width * 0.5); // |omega|=v/r_left
        const double center_tangential = curvature * angular_speed; // v=|omega|*r_center
        double correction_ratio = max_velocity0 / fabs(center_tangential);
        correction_ratio = correction_ratio < 1.0 ? (correction_ratio > 0.2 ? correction_ratio : 0.2) : 1.0;
        left_velocity *= correction_ratio;
        right_velocity *= correction_ratio;
        double least_voltage = fabs(left_velocity) < fabs(right_velocity) ? fabs(left_velocity) : fabs(right_velocity);
        if (least_voltage < min_voltage) {
            correction_ratio = min_voltage / least_voltage;
            left_velocity *= correction_ratio;
            right_velocity *= correction_ratio;
        }
        #endif
        steer(left_velocity, right_velocity);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    left -> stop(vex::brakeType::coast);
    right -> stop(vex::brakeType::coast);
    printf("done!\n");
}

void Chassis::turn_to(double angle, double tolerance, double maximum, double minimum, double activation_threshold, double integral_threshold, double derivative_threshold, double p, double i, double d) {
    double target = floor((rotation() - angle + M_PI) / (M_PI * 2)) * M_PI * 2 + angle;
    double error0 = target - rotation();
    turn(error0, tolerance, maximum, minimum, activation_threshold, integral_threshold, derivative_threshold, p, i, d);
}

void Chassis::turn(double angle, double tolerance, double maximum, double minimum, double activation_threshold, double integral_threshold, double derivative_threshold, double p, double i, double d) {
    printf("Called turn... ");
    double target = rotation() + angle;
    PID turn_controller(p, i, d, // adjust gains // 5 0.3 5
                        target, // target position
                        tolerance, // tolerance: allowed error
                        maximum, // max_value
                        minimum, // min_value
                        activation_threshold,
                        integral_threshold,
                        derivative_threshold, // derivative_threshold
                        50.0, // max_integral
                        0.999); // gamma
    while (!turn_controller.arrived()) {
        double pos = rotation();
        double output = turn_controller.calculate(pos);
        left -> spin(-output, vex::voltageUnits::volt);
        right -> spin(output, vex::voltageUnits::volt);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    left -> stop(vex::brakeType::coast);
    right -> stop(vex::brakeType::coast);
    printf("done!\n");
}

void Chassis::forward(double distance, double tolerance, double maximum, double minimum, double activation_threshold, double integral_threshold, double derivative_threshold, double p, double i, double d) {
    double target_x = x() + distance * cos(rotation());
    double target_y = y() + distance * sin(rotation());
    printf("Calling forward from (%.5f, %.5f, %.5f) -> (%.5f, %.5f)... \n", x(), y(), rotation(), target_x, target_y);
    double original_x = x();
    double original_y = y();
    auto dist = [this, distance, original_x, original_y] () {
        double dx = x() - original_x;
        double dy = y() - original_y;
        double raw_distance = sqrt(dx * dx + dy * dy);        
        return distance < 0 ? -raw_distance : raw_distance;
    };
    PID forward_controller(p, i, d, // adjust gains
                           distance, // target position
                           tolerance, // tolerance: allowed error
                           maximum, // max_value
                           minimum, // min_value
                           activation_threshold,
                           integral_threshold,
                           derivative_threshold, // derivative_threshold
                           50.0, // max_integral
                           0.999); // gamma
    PID angular_controller(15.0, 0.9, 45.0,
                           rotation(), // target position
                           0.0, // tolerance: allowed error
                           6.0, // max_value
                           0.0, // min_value
                           1e9, // activation_threshold
                           M_PI / 10, // integration_threshold
                           0.0, // derivative_threshold
                           10.0, // max_integral
                           0.999); // gamma
    while (!forward_controller.arrived()) {
        double pos = dist();
        double output = forward_controller.calculate(pos);
        double progress = fabs(pos / distance);
        // double angle_filter = 1 / (1 + exp((progress - 0.75) * 30));
        double angle_filter = 1 / (1 + exp((progress - 0.5) * 30)); // \frac{1}{1+e^{30\left(x-0.5\right)}} - logistic function
        double relative_angle;
        if (distance > 0) {
            relative_angle = atan2(target_y - y(), target_x - x());
        } else {
            relative_angle = atan2(y() - target_y, x() - target_x);
        }
        double actual_relative = floor((rotation() - relative_angle + M_PI) / (M_PI * 2)) * M_PI * 2 + relative_angle;
        double correction = angular_controller.calculate(actual_relative) * angle_filter;
        double left_voltage, right_voltage;
        if (distance > 0) {
            left_voltage = output + correction;
            right_voltage = output - correction;
        } else {
            left_voltage = output + correction;
            right_voltage = output - correction;
        }
        left -> spin(left_voltage, vex::voltageUnits::volt);
        right -> spin(right_voltage, vex::voltageUnits::volt);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    left -> stop(vex::brakeType::coast);
    right -> stop(vex::brakeType::coast);
    printf("done!\n");
}

void Chassis::forward_timer(double time, double base_voltage, double corrective_strength) {
    left -> spin(base_voltage, vex::voltageUnits::volt);
    right -> spin(base_voltage, vex::voltageUnits::volt);
    double original_rotation = rotation();
    PID angular_correction(corrective_strength * 15.0, corrective_strength * 0.9, corrective_strength * 45.0,
                           original_rotation, // target position
                           0.0, // tolerance: allowed error
                           3.0, // max_value
                           0.0, // min_value
                           1e9, // activation_threshold
                           1.0, // integration_threshold
                           0.0, // derivative_threshold
                           10.0, // max_integral
                           0.999); // gamma
    auto start = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - start).count();
    while (duration < time) {
        // double relative_angle = original_rotation - rotation();
        double correction = angular_correction.calculate(rotation());
        // Only start limiting the last second
        double remaining_time = duration - time + 1;
        remaining_time = remaining_time < 0 ? 0 : remaining_time;
        double adjusted_voltage = base_voltage / (1 + exp((remaining_time - 0.85) * 10));
        double adjusted_correction = correction / (1 + exp((remaining_time - 0.75) * 20));
        double left_voltage = adjusted_voltage - adjusted_correction;
        double right_voltage = adjusted_voltage + adjusted_correction;
        left -> spin(left_voltage, vex::voltageUnits::volt);
        right -> spin(right_voltage, vex::voltageUnits::volt);
        duration = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - start).count();
    }
    left -> stop();
    right -> stop();
    printf("Done\n");
}

void Chassis::steer_timer(double left_voltage, double right_voltage, double time) {
    printf("Called steer_timer... ");
    left -> spin(left_voltage, vex::voltageUnits::volt);
    right -> spin(right_voltage, vex::voltageUnits::volt);
    vexDelay(time * 1000);
    left -> stop();
    right -> stop();
    printf("done!\n");
}

void Chassis::drift_in_place(double bias, double new_heading, double strength, double maximum, double minimum, double activation_ratio, double integral_ratio) {
    PID phase1_angular(strength * 15.0, strength * 0.9, strength * 45.0,
                       new_heading, // target position
                       0.035, // tolerance: allowed error
                       maximum, // max_value
                       minimum, // min_value
                       fabs(bias * activation_ratio), // activation_threshold
                       fabs(bias * integral_ratio), // integration_threshold
                       0.05, // derivative_threshold
                       10.0, // max_integral
                       0.999); // gamma
    double original_rotation = rotation();
    while (!phase1_angular.arrived()) {
        double output = phase1_angular.calculate(rotation());
        double linear = bias;
        double angle_progress = fabs((rotation() - original_rotation) / (new_heading - original_rotation));
        double angle_filter = 1 / (1 + exp((angle_progress - 0.8) * 10));
        linear *= angle_filter;
        left -> spin(linear - output, vex::voltageUnits::volt);
        right -> spin(linear + output, vex::voltageUnits::volt);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    left -> stop();
    right -> stop();
    vexDelay(100);
    turn_to(new_heading, 0.05, maximum, minimum, activation_ratio, integral_ratio);
}

void Chassis::drift_after_distance(double distance, double bias, double new_heading, double strength, double maximum, double minimum, double activation_ratio, double integral_ratio) {
    double original_x = x(), original_y = y();
    double original_rotation = rotation();
    double target = fmod(rotation(), M_PI * 2);
    if (target < 0) {
        target += M_PI * 2;
    }
    PID phase1_angular(15.0, 0.9, 45.0,
                       target, // target position
                       0.0, // tolerance: allowed error
                       3.0, // max_value
                       0.0, // min_value
                       fabs(distance * activation_ratio), // activation_threshold
                       fabs(distance * integral_ratio), // integration_threshold
                       0.0, // derivative_threshold
                       10.0, // max_integral
                       0.999); // gamma
    auto dist = [this, original_x = x(), original_y = y()] () {
        double dx = x() - original_x;
        double dy = y() - original_y;
        return sqrt(dx * dx + dy * dy);
    };
    while (dist() < distance) {
        double output = 12.0;
        double relative_angle = atan2(original_y - y(), original_x - x());
        double correction = phase1_angular.calculate(relative_angle);
        left -> spin(output + correction, vex::voltageUnits::volt);
        right -> spin(output - correction, vex::voltageUnits::volt);
    }
    PID phase2_angular(strength * 15.0, strength * 0.9, strength * 45.0,
                       new_heading, // target position
                       0.035, // tolerance: allowed error
                       maximum, // max_value
                       minimum, // min_value
                       fabs(distance * activation_ratio), // activation_threshold
                       fabs(distance * integral_ratio), // integration_threshold
                       0.05, // derivative_threshold
                       10.0, // max_integral
                       0.999); // gamma
    while (!phase2_angular.arrived()) {
        double output = phase2_angular.calculate(rotation());
        double linear = bias;
        double angle_progress = fabs((rotation() - original_rotation) / (new_heading - original_rotation));
        double angle_filter = 1 / (1 + exp((angle_progress - 0.8) * 10));
        linear *= angle_filter;
        left -> spin(linear - output, vex::voltageUnits::volt);
        right -> spin(linear + output, vex::voltageUnits::volt);
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    left -> stop();
    right -> stop();
    vexDelay(100);
    turn_to(new_heading, 0.05, maximum, minimum, activation_ratio, integral_ratio);
}

void Chassis::corner_reset(double lengthwise_offset) {
    double sine = sin(rotation());
    double cosine = cos(rotation());
    double corner_x, corner_y;
    corner_x = cosine > 0 ? 0 : 144;
    corner_y = sine > 0 ? 0 : 144;
    double pt1_x = corner_x + base_width * sine;
    double pt1_y = corner_y;
    double pt2_x = corner_x;
    double pt2_y = corner_y + base_width * cosine;
    double midpt_x = (pt1_x + pt2_x) * 0.5;
    double midpt_y = (pt1_y + pt2_y) * 0.5;
    double dx = lengthwise_offset * cosine;
    double dy = lengthwise_offset * sine;
    double new_x = midpt_x + dx;
    double new_y = midpt_y + dy;
    set_pose(new_x, new_y, rotation());
    printf("Corner reset to (%.5f, %.5f, %.5f)\n", new_x, new_y, rotation() / M_PI * 180.0);
}

void Chassis::corner_reset_forward(double lengthwise_offset) {
    double sine = sin(rotation());
    double cosine = cos(rotation());
    double corner_x, corner_y;
    
    // Corner selection for front-facing (inverted from back-facing logic)
    corner_x = cosine < 0 ? 0 : 144;
    corner_y = sine < 0 ? 0 : 144;
    
    // Calculate points along the walls with negated signs
    double pt1_x = corner_x - base_width * sine;
    double pt1_y = corner_y;
    double pt2_x = corner_x;
    double pt2_y = corner_y - base_width * cosine;
    double midpt_x = (pt1_x + pt2_x) * 0.5;
    double midpt_y = (pt1_y + pt2_y) * 0.5;
    
    // Offset in opposite direction of heading
    double dx = -lengthwise_offset * cosine;
    double dy = -lengthwise_offset * sine;
    
    double new_x = midpt_x + dx;
    double new_y = midpt_y + dy;
    set_pose(new_x, new_y, rotation());
    printf("Corner reset to (%.5f, %.5f, %.5f)\n", new_x, new_y, rotation() / M_PI * 180.0);
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

int wire_control(void* o) {
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

int arcade_control(void* o) {
    Chassis* chassis = static_cast<Chassis*>(o);
    vex::controller& controller = *(chassis -> controller);
    MotorGroup& left = *(chassis -> left);
    MotorGroup& right = *(chassis -> right);
    while (true) {
        double linear_power = controller.Axis3.position(vex::percentUnits::pct);
        double turn_power = controller.Axis1.position(vex::percentUnits::pct);
        double left_voltage = (linear_power + turn_power) * 12.8 * 128;
        double right_voltage = (linear_power - turn_power) * 12.8 * 128;
        left.spin(left_voltage, vex::voltageUnits::mV);
        right.spin(right_voltage, vex::voltageUnits::mV);
        vex::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}

int curvature_control(void* o) {
    Chassis* chassis = static_cast<Chassis*>(o);
    vex::controller& controller = *(chassis -> controller);
    MotorGroup& left = *(chassis -> left);
    MotorGroup& right = *(chassis -> right);
    while (true) {
        // Get raw joystick values (-1.0 to 1.0)
        double throttle = controller.Axis3.position() / 100.0;
        double turn = controller.Axis1.position() / 100.0;
        
        // Apply deadband
        const double DEADBAND = 0.05;
        if (fabs(throttle) < DEADBAND) throttle = 0.0;
        if (fabs(turn) < DEADBAND) turn = 0.0;
        
        double left_power, right_power;
        
        if (fabs(throttle) < DEADBAND && fabs(turn) > DEADBAND) {
            // Turn in place
            left_power = turn;
            right_power = -turn;
        } else {
            // Curvature drive
            const double SENSITIVITY = 0.8; // Adjust this to change turning sensitivity
            double angularCmd = throttle * turn * SENSITIVITY;
            
            left_power = throttle + angularCmd;
            right_power = throttle - angularCmd;
            
            // Normalize to prevent motor saturation
            double maxMagnitude = std::max(fabs(left_power), fabs(right_power));
            if (maxMagnitude > 1.0) {
                left_power /= maxMagnitude;
                right_power /= maxMagnitude;
            }
        }
        
        // Send to motors
		left.spin(left_power * 12.0, vex::voltageUnits::volt);
		right.spin(right_power * 12.0, vex::voltageUnits::volt);
        
        // Small delay to prevent CPU overload
        vex::this_thread::sleep_for(10);
    }
    return 0;
}

int tank_control(void* o) {
    Chassis* chassis = static_cast<Chassis*>(o);
    vex::controller& controller = *(chassis -> controller);
    MotorGroup& left = *(chassis -> left);
    MotorGroup& right = *(chassis -> right);
	while (true) {
		double left_power = controller.Axis3.position() / 100.0;
		double right_power = controller.Axis2.position() / 100.0;

		left.spin(left_power * 12.0, vex::voltageUnits::volt);
		right.spin(right_power * 12.0, vex::voltageUnits::volt);

		vex::this_thread::sleep_for(10);
	}
}