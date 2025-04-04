#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include <motor.hpp>
#include <motor_group.hpp>
#include <odometry.hpp>
#include <path.hpp>
#include <pursuit.hpp>
#include <pid.hpp>
#include <cmath>
#include <math.h>
#include <algorithm>

#define M_PI 3.14159265358979323846

class Chassis {
    MotorGroup* left;
    MotorGroup* right;
    vex::rotation* left_track;
    vex::rotation* back_track;
    vex::inertial* imu;
    Odometry* odometry;
    double base_width;
    double left_offset;
    double back_offset;
    double wheel_radius;
    double tracking_radius;
    double external_ratio;
    double max_radial_acceleration;
public:
    Chassis(MotorGroup* left, MotorGroup* right, vex::rotation* left_track, vex::rotation* back_track, vex::inertial* imu, double base_width, double left_offset, double back_offset, double wheel_radius, double tracking_radius, double external_ratio, double max_radial_acceleration);

    void steer(double left_voltage, double right_voltage);
    void set_pose(double x, double y, double rotation);
    
    void follow_path(Path path, double tolerance, double lookahead);
    void turn_to(double angle, double tolerance, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2);
    void turn(double angle, double tolerance, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2);

    void forward(double distance, double correction_strength, double tolerance = 0.05, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2);

    void reset_position();
    void stop();
    double x();
    double y();
    double rotation();
};

#endif // #ifndef CHASSIS_HPP