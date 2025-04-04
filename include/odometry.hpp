#ifndef VEXLIBRARY_VIRTUAL_ODOMETRY_HPP
#define VEXLIBRARY_VIRTUAL_ODOMETRY_HPP

#include <vex.h>
#include <motor_group.hpp>
#include <vector>
#include <cmath>

class Odometry {
    friend int track(void* o);
    friend int display(void* o);
    vex::thread tracking_thread;
    vex::thread displaying_thread;
    vex::rotation* forward_track;
    vex::rotation* side_track;
    vex::inertial* imu;
    double left_right_offset; // left is positive
    double forward_back_offset; // back is positive
    double tracking_radius;
    double x_position;
    double y_position;
    double rotation_value;
    double get_forward();
    double get_back();
public:
    Odometry(vex::rotation* forward_track, vex::rotation* left_track, vex::inertial* imu,
             double left_right_offset,  double forward_back_offset, double tracking_radius);
    double x();
    double y();
    double rotation();
    void reset();
    void set_pose(double x, double y, double rotation);
};

// class Odometry {
//     friend int track(void * o);
//     friend int display(void * o);
//     vex::thread tracking_thread;
//     vex::thread displaying_thread;
//     MotorGroup* left;
//     MotorGroup* right;
//     vex::rotation * back;
//     double base_width;
//     double back_offset;
//     double main_radius;
//     double drift_radius;
//     double x_position;
//     double y_position;
//     double rotation_value;
//     double get_left();
//     double get_right();
//     double get_back();
// public:
//     Odometry(MotorGroup* left, MotorGroup* right, vex::rotation* back, double base_width, double back_offset, double main_radius, double drift_radius);
//     double x();
//     double y();
//     double rotation();
//     void reset();
//     void set_pose(double x, double y, double rotation);
// };

#endif // #ifndef VEXLIBRARY_VIRTUAL_ODOMETRY_HPP