#pragma once
#ifndef MOTOR_GROUP_HPP
#define MOTOR_GROUP_HPP

#include <vex.h>
#include <motor.hpp>
#include <vector>

class MotorGroup {
    double cartridge_ratio;
    double external_ratio;
    double wheel_radius;
    double gear_ratio(vex::gearSetting gears) {
        switch (gears) {
            case vex::gearSetting::ratio6_1:
                return 1 / 6.0;
            case vex::gearSetting::ratio18_1:
                return 1 / 18.0;
            case vex::gearSetting::ratio36_1:
                return 1 / 36.0;
        }
        return 1;
    }
    void add_motor(Motor* motor) {
        motors.push_back(motor);
    }
    template <typename... Args>
    void add_motor(Motor* motor, Args... args) {
        motors.push_back(motor);
        add_motor(args...);
    }
public:
    std::vector<Motor*> motors;
    MotorGroup(double external_ratio, double wheel_radius, Motor* motor): motors(), cartridge_ratio(1), external_ratio(external_ratio), wheel_radius(wheel_radius) {
        add_motor(motor);
    }
    template <typename... Args>
    MotorGroup(double external_ratio, double wheel_radius, Motor* motor, Args... args): motors(), cartridge_ratio(1), external_ratio(external_ratio), wheel_radius(wheel_radius) {
        add_motor(args...);
    }
    Motor& operator [] (int idx) const {
        return *motors[idx];
    }
    void spin(double velocity) {
        for (Motor* motor: motors) {
            motor -> spin(velocity);
        }
    }
    void spin(double power, vex::percentUnits units) {
        for (Motor* motor: motors) {
            motor -> spin(power, units);
        }
    }
    void spin(double voltage, vex::voltageUnits units) {
        for (Motor* motor: motors) {
            motor -> spin(voltage, units);
        }
    }
    void reset() {
        for (Motor* motor: motors) {
            motor -> reset_position();
        }
    }
    void stop() {
        for (Motor* motor: motors) {
            motor -> stop();
        }
    }
    int connected() const {
        // return bitset of connected motors
        int result = 0, idx = 0;
        for (Motor* motor: motors) {
            result |= ((motor -> connected() ? 1 : 0) << (idx++));
        }
        return result;
    }
    int number_connected() const {
        int result = 0;
        for (Motor* motor: motors) {
            if (motor -> connected()) {
                ++result;
            }
        }
        return result;
    }
    double device_position(vex::rotationUnits units) const {
        double result = 0;
        for (Motor* motor: motors) {
            result += motor -> device_position(units);
        }
        return result / (double) number_connected();
    }
    double wheel_position(vex::rotationUnits units) const {
        double result = 0;
        for (Motor* motor: motors) {
            result += motor -> wheel_position(units);
        }
        return result / (double) number_connected();
    }
    double max_wheel_velocity() const {
        return motors[0] -> max_wheel_velocity();
    }
};

#endif // #ifndef MOTOR_GROUP_HPP