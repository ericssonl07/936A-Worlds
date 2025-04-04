#include <chassis.hpp>

Chassis::Chassis(MotorGroup* left, MotorGroup* right, vex::rotation* left_track, vex::rotation* back_track, vex::inertial* imu,
                 double base_width, double left_offset, double back_offset, double wheel_radius, double tracking_radius, double external_ratio, double max_radial_acceleration)
    : left(left), right(right), left_track(left_track), back_track(back_track), imu(imu), base_width(base_width),
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
        auto [steering, curvature] = pursuit.get_relative_steering(x(), y(), rotation(), base_width, 12.0);
        auto [left_velocity, right_velocity] = steering;
        
        // auto [target_x, target_y] = pursuit.get_target(x(), y());
        // printf("(%.5f, %.5f) -> (%.5f, %.5f): (%.5f, %.5f)\n\n\n", x(), y(), target_x, target_y, left_velocity, right_velocity); vexDelay(1);
        printf("(%.5f, %.5f)\n", x(), y());

        // #define RESTRICT_VELOCITY // Uncomment this line to restrict the velocity
        #ifdef RESTRICT_VELOCITY
        const double max_velocity = sqrt(fabs(curvature * max_radial_acceleration)); // v=√(ar)
        const double v_wheel_max = left -> max_wheel_velocity();
        const double left_tangential_as_pct_max = left_velocity / 12.0;
        const double left_tangential = v_wheel_max * left_tangential_as_pct_max;
        const double angular_speed = left_tangential / (curvature - base_width * 0.5); // |omega|=v/r_left
        const double center_tangential = curvature * angular_speed; // v=|omega|*r_center
        const double correction_ratio = std::min(max_velocity / center_tangential, 1.0);
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