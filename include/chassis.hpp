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
#include <functional>

#define M_PI 3.14159265358979323846

class Chassis {
    MotorGroup* left;
    MotorGroup* right;
    vex::rotation* left_track;
    vex::rotation* back_track;
    vex::inertial* imu;
    vex::controller* controller;
    Odometry* odometry;
    double base_width;
    double left_offset;
    double back_offset;
    double wheel_radius;
    double tracking_radius;
    double external_ratio;
    double max_radial_acceleration;
public:
    Chassis(MotorGroup* left, MotorGroup* right, vex::rotation* left_track, vex::rotation* back_track, vex::inertial* imu, vex::controller* controller, double base_width, double left_offset, double back_offset, double wheel_radius, double tracking_radius, double external_ratio, double max_radial_acceleration);

    void steer(double left_voltage, double right_voltage);
    void set_pose(double x, double y, double rotation);
    
    void follow_path(Path path, double tolerance, double lookahead); // Tested on Friday, April 11th, 2025
    void turn_to(double angle, double tolerance, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2); // Tested on Friday, April 11th, 2025
    void turn(double angle, double tolerance, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2); // Tested on Friday, April 11th, 2025

    void forward(double distance, double tolerance = 0.05, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2); // Tested on Friday, April 11th, 2025
    void forward_timer(double time, double base_voltage = 12.0, double corrective_strength = 1.0); // Tested on Friday, April 11th, 2025
    void steer_timer(double left_voltage, double right_voltage, double time); // Tested on Friday, April 11th, 2025
    void drift_in_place(double bias, double new_heading, double strength, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2); // Tested on Friday, April 11th, 2025
    void drift_after_distance(double distance, double bias, double new_heading, double strength, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2); // Tested on Friday, April 11th, 2025
    void corner_reset(double lengthwise_offset); // Tested on Friday, April 11th, 2025

    void reset_position();
    void stop();
    double x();
    double y();
    double rotation();
    friend int wire_control(void* o); // Tested on Friday, April 11th, 2025
    friend int arcade_control(void* o); // Tested on Friday, April 11th, 2025
    friend int curvature_control(void* o); // Tested on Friday, April 11th, 2025
    friend int tank_control(void* o); // Tested on Friday, April 11th, 2025
};

int wire_control(void* o);
int arcade_control(void* o);
int curvature_control(void* o);
int tank_control(void* o);

#endif // #ifndef CHASSIS_HPP