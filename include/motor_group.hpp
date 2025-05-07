#pragma once
#ifndef MOTOR_GROUP_HPP
#define MOTOR_GROUP_HPP

#include <vex.h>
#include <motor.hpp>
#include <vector>

/**
 * @class MotorGroup
 * @file motor_group.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This class represents a group of motors and provides methods for controlling their behavior.
 * It allows for easy manipulation of multiple motors as a single unit, including spinning, stopping, and resetting their positions.
 * The class also provides methods for checking the connection status of the motors and calculating their average position.
 * @details
 * The MotorGroup class is designed to work with a set of motors, allowing for easy control and management of multiple motors at once.
 * It provides methods for adding motors to the group, spinning them at a specified velocity, and stopping them.
 * The class also includes methods for resetting the positions of the motors and checking their connection status.
 * Provided the external ratio and wheel radius, the class can calculate the motor's internal (raw), shaft, and wheel properties (e.g., velocity, position).
 * @attention
 * The class expects that all managed devices have the same external ratio and radius, and that they are coupled (e.g., in a typical VEX [non-XDrive] base).
 * 
 * The class also expects external ratios to be given as the ratio of output spin to input spin. 
 * For instance, if the motor spins 6 times for every 1 turn of the wheel, the external ratio should be set
 * to 1/6 as the external wheel turns 1/6 times for every 1 turn of the motor.
 * There is no need to account for the cartridge, which vex::motor does automatically.
 * @note
 * The MotorGroup class is implemented using a vector to store the motors, allowing for dynamic addition and removal of motors.
 * The class provides a flexible interface for controlling the motors, making it easy to integrate into various robotics applications.
 * Internally, it is essentially a wrapper around the Motor class, providing additional functionality for managing multiple motors.
 * @example
 * ```cpp
 * // Example usage of the MotorGroup class:
 * Motor motor1(vex::PORT1, vex::gearSetting::ratio18_1, false);
 * Motor motor2(vex::PORT2, vex::gearSetting::ratio18_1, false);
 * MotorGroup motorGroup(1.0, 2.0, &motor1, &motor2); // Create a MotorGroup with two motors, external ratio 1.0 and wheel radius 2.0
 * motorGroup.spin(50); // Spin all motors at 50 in/s for wheel tangential velocity
 * motorGroup.spin(12.0, vex::voltageUnits::volt); // Spin all motors at 12V
 * motorGroup.spin(50, vex::percentUnits::pct); // Spin all motors at 50% power (internal calculation to 6V)
 * motorGroup.stop(); // Stop all motors
 * motorGroup.reset(); // Reset the position of all motors
 * int k = motorGroup.connected(); // Check the connection status of all motors as a bitset
 * int n = motorGroup.number_connected(); // Get the number of connected motors
 * for (int i = 0; i < n; ++i) {
 *   connected_status = k & (1 << i);
 *   printf("Motor %d connected: %d\n", i, connected_status);
 * }
 * double avg_position = motorGroup.device_position(vex::rotationUnits::deg); // Get the average position of all motors in degrees
 * double avg_wheel_position = motorGroup.wheel_position(vex::rotationUnits::deg); // Get the average wheel position of all motors in degrees
 * ```
 * @see
 * Motor
 * 
 * vex::motor
 */
class MotorGroup {

/**
 * @privatesection
 */
private:

    /**
     * @private cartridge_ratio
     * @brief Gear ratio of the motor cartridge
     * @note The cartridge ratio is one of the following:
     * 
     * - 1/6: blue cartridge
     * - 1/18: green cartridge
     * - 1/36: red cartridge
     */
    double cartridge_ratio;

    /**
     * @private external_ratio
     * @brief External gear ratio of the robot's motors
     * @note The external gear ratio is the ratio of the number of teeth on the driven gear to the number of teeth on the driving gear.
     * This value is used to calculate the speed and torque of the motors. The gear ratio is given as the ratio of output spin to input spin.
     * For instance, if the motor spins 6 times for every 1 turn of the wheel, the external ratio should be set to 1/6 as the external
     * wheel turns 1/6 times for every 1 turn of the shaft. There is no need to account for the cartridge, which vex::motor does automatically.
     */
    double external_ratio;

    /**
     * @private wheel_radius
     * @brief Radius of the robot's wheels
     * @note The wheel radius is the distance from the center of the wheel to its outer edge, in inches.
     * The official VEX radius is not the exact radius of the wheel; it's recommended to measure the radius of the wheel with a caliper.
     */
    double wheel_radius;

    /**
     * @private gear_ratio
     * @brief Get the gear ratio of the motor cartridge
     * @param gears Gear setting of the motor cartridge
     * @note The gear ratio is one of the following:
     * 
     * - 1/6: blue cartridge
     * - 1/18: green cartridge
     * - 1/36: red cartridge
     * @returns The gear ratio of the motor cartridge
     */
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

    /**
     * @private add_motor
     * @brief Add a motor to the group
     * @param motor Motor to add
     * @note This method is used internally to add motors to the group. It is a specialized method 
     * of the template variant that takes a single motor as an argument.
     */
    void add_motor(Motor* motor) {
        motors.push_back(motor);
    }

    /**
     * @private add_motor
     * @brief Add multiple motors to the group
     * @param motor Motor to add
     * @param args Additional motors to add
     * @note This method is used internally to add multiple motors to the group.
     */
    template <typename... Args>
    void add_motor(Motor* motor, Args... args) {
        motors.push_back(motor);
        add_motor(args...);
    }

/**
 * @publicsection
 */
public:

    /**
     * @public motors
     * @brief Vector of motors in the group
     */
    std::vector<Motor*> motors;

    /**
     * @public constructor
     * @brief Constructor for the MotorGroup class
     * @param external_ratio External gear ratio of the robot's motors
     * @param wheel_radius Radius of the robot's wheels
     * @param motor Motor to add to the group
     * @note This constructor is used to create a MotorGroup with a single motor. It is a
     * specialized constructor for the case where only one motor is provided.
     * @attention Motors should be coupled, with equal external ratios and wheel radii.
     */
    MotorGroup(double external_ratio, double wheel_radius, Motor* motor): motors(), cartridge_ratio(1), external_ratio(external_ratio), wheel_radius(wheel_radius) {
        add_motor(motor);
    }

    /**
     * @public constructor
     * @brief Constructor for the MotorGroup class
     * @param external_ratio External gear ratio of the robot's motors
     * @param wheel_radius Radius of the robot's wheels
     * @param motor Motor to add to the group
     * @param args Additional motors to add to the group
     * @note This constructor is used to create a MotorGroup with multiple motors.
     * It is a template constructor that takes a variable number of motors as arguments.
     * @attention Motors should be coupled, with equal external ratios and wheel radii.
     
     */
    template <typename... Args>
    MotorGroup(double external_ratio, double wheel_radius, Motor* motor, Args... args): motors(), cartridge_ratio(1), external_ratio(external_ratio), wheel_radius(wheel_radius) {
        add_motor(args...);
    }

    /**
     * @public operator[]
     * @brief Access a motor in the group
     * @param idx Index of the motor to access
     * @returns A reference to the specified motor in the group
     */
    Motor& operator [] (int idx) const {
        return *motors[idx];
    }

    /**
     * @public spin
     * @brief Spin all motors in the group
     * @param velocity Velocity to spin the motors at (in/s external tangential velocity)
     * @attention Motors should be coupled, with equal external ratios and wheel radii.
     */
    void spin(double velocity) {
        for (Motor* motor: motors) {
            motor -> spin(velocity);
        }
    }

    /**
     * @public spin
     * @brief Spin all motors in the group
     * @param power Power to spin the motors at (percentage of maximum power)
     * @param units Units of the power (percentage or voltage)
     * @attention Motors should be coupled, with equal external ratios and wheel radii.
     */
    void spin(double power, vex::percentUnits units) {
        for (Motor* motor: motors) {
            motor -> spin(power, units);
        }
    }

    /**
     * @public spin
     * @brief Spin all motors in the group
     * @param voltage Voltage to spin the motors at (volts)
     * @param units Units of the voltage (volts or percentage)
     * @attention Motors should be coupled, with equal external ratios and wheel radii.
     */
    void spin(double voltage, vex::voltageUnits units) {
        for (Motor* motor: motors) {
            motor -> spin(voltage, units);
        }
    }

    /**
     * @public reset
     * @brief Reset the position of all motors in the group
     */
    void reset() {
        for (Motor* motor: motors) {
            motor -> reset_position();
        }
    }

    /**
     * @public stop
     * @brief Stop all motors in the group
     * @note This method stops all motors in the group immediately and locks them.
     * It uses vex::brakeType::brake to stop the motors.
     */
    void stop(vex::brakeType brake_mode = vex::brakeType::coast) {
        for (Motor* motor: motors) {
            motor -> stop(brake_mode);
        }
    }

    /**
     * @public connected
     * @brief Check the connection status of all motors in the group
     * @returns An integer representing the connection status of each motor in the group in its bits
     */
    int connected() const {
        // return bitset of connected motors
        int result = 0, idx = 0;
        for (Motor* motor: motors) {
            result |= ((motor -> connected() ? 1 : 0) << (idx++));
        }
        return result;
    }

    /**
     * @public number_connected
     * @brief Get the number of connected motors in the group
     * @returns The number of connected motors in the group
     */
    int number_connected() const {
        int result = 0;
        for (Motor* motor: motors) {
            if (motor -> connected()) {
                ++result;
            }
        }
        return result;
    }

    /**
     * @public device_position
     * @brief Get the average raw position of all motors in the group
     * @param units Units of the position (degrees, radians, etc.)
     * @attention Motors should have been reset at the same time, or else this reading is meaningless.
     * @returns The average raw position of all motors in the group in the specified units
     */
    double device_position(vex::rotationUnits units) const {
        double result = 0;
        for (Motor* motor: motors) {
            result += motor -> device_position(units);
        }
        return result / (double) number_connected();
    }

    /**
     * @public wheel_position
     * @brief Get the average wheel position of all motors in the group
     * @param units Units of the position (degrees, radians, etc.)
     * @attention Motors should have been reset at the same time, or else this reading is meaningless.
     * @returns The average wheel position of all motors in the group in the specified units
     */
    double wheel_position(vex::rotationUnits units) const {
        double result = 0;
        for (Motor* motor: motors) {
            result += motor -> wheel_position(units);
        }
        return result / (double) number_connected();
    }

    /**
     * @public max_wheel_velocity
     * @brief Get the maximum wheel velocity of the first motor in the group
     * @note Since all motors are coupled with the same external ratio, cartridge, and wheel radius, the maximum wheel velocity is the same for all motors.
     * @returns The maximum wheel velocity of the first motor in the group
     */
    double max_wheel_velocity() const {
        return motors[0] -> max_wheel_velocity();
    }
};

#endif // #ifndef MOTOR_GROUP_HPP