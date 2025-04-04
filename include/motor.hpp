#pragma once
#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <vex.h>
#include <cmath>
#include <utility>
#include <pid.hpp>

// If gear ratio is 6:1, the motor spins 6 times for every 1 turn of the wheel- set ratio to 1/6
// Thus, the wheel spins 1/6 times for every 1 turn of the motor- this is our ratio definition

#define M_PI 3.14159265358979323846

class Motor {
    static constexpr const double max_motor_rpm = 3600;
    static constexpr const double max_voltage = 12; // 11.76
    vex::motor* device;
    double cartridge_ratio;
    double external_ratio;
    double wheel_radius;
    double spin_to_impl_position, spin_to_impl_tolerance, spin_to_impl_maximum, spin_to_impl_minimum, spin_to_impl_activation_ratio, spin_to_impl_integral_ratio;
    double gear_ratio(vex::gearSetting gears);
    static void spin_to_impl(void* instance);
public:
    explicit Motor(vex::motor* device, double external_ratio, double wheel_radius);
    explicit Motor(int32_t index, vex::gearSetting gears, bool reverse, double external_ratio, double wheel_radius);
    explicit Motor(const Motor&);
    explicit Motor(Motor&&);
    Motor& operator=(const Motor&);
    Motor& operator=(Motor&&);
    ~Motor();
    void spin(double velocity);
    void spin(double power, vex::percentUnits units);
    void spin(double voltage, vex::voltageUnits units);
    void reset_position();
    void stop();
    bool connected();
    double device_position(vex::rotationUnits units) const;
    double wheel_position(vex::rotationUnits units) const;
    double max_wheel_velocity() const;
    void spin_to(double position, double tolerance = 2.0, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2, bool block = true);
};

#endif // #ifndef MOTOR_HPP