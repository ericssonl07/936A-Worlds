#pragma once
#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <vex.h>
#include <cmath>
#include <utility>
#include <pid.hpp>

#define M_PI 3.14159265358979323846

/**
 * @class Motor
 * @file motor.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This class represents a motor and provides methods for controlling its behavior.
 * @details
 * The Motor class is designed to work with a VEX motor and provides methods for spinning the motor, resetting its position, and checking its connection status.
 * It allows for easy manipulation of the motor's speed, voltage, and position.
 * The class is an abstraction on top of vex::motor that accounts for gear ratios.
 * @attention
 * The gear ratio is expected to be given as the ratio of output spin to input spin. 
 * For instance, if the motor spins 6 times for every 1 turn of the wheel, the external ratio should be set
 * to 1/6 as the external wheel turns 1/6 times for every 1 turn of the motor.
 * There is no need to account for the cartridge, which vex::motor does automatically.
 * @example
 * ```cpp
 * // Example usage of the Motor class:
 * Motor motor(vex::PORT1, vex::gearSetting::ratio18_1, false, 1.0, 2.0); // Create a motor with external ratio 1.0 and wheel radius 2.0
 * motor.spin(50); // Spin the motor at 50 in/s for wheel tangential velocity
 * motor.spin(12.0, vex::voltageUnits::volt); // Spin the motor at 12V
 * motor.spin(50, vex::percentUnits::pct); // Spin the motor at 50% power (internal calculation to 6V)
 * motor.stop(); // Stop the motor
 * motor.reset_position(); // Reset the position of the motor
 * bool connected_status = motor.connected(); // Check the connection status of the motor
 * double position = motor.device_position(vex::rotationUnits::deg); // Get the position of the motor in degrees
 * double wheel_position = motor.wheel_position(vex::rotationUnits::deg); // Get the wheel position of the motor in degrees
 * double max_velocity = motor.max_wheel_velocity(); // Get the maximum wheel velocity of the motor
 * motor.spin_to(90.0); // Spin the motor to 90 degrees
 * motor.spin_to(90.0, 2.0, 12.0, 0.02, 0.3, 0.2); // Spin the motor to 90 degrees with specified parameters- blocking execution
 * motor.spin_to(90.0, 2.0, 12.0, 0.02, 0.3, 0.2, false); // Spin the motor to 90 degrees with specified parameters- non-blocking execution
 * ```
 * @see
 * vex::motor
 * MotorGroup
 */
class Motor {

/**
 * @privatesection
 */
private:

    /**
     * @private max_motor_rpm
     * @brief Maximum RPM of the motor- equals 3600 rpm for a VEX V5 Smart Motor
     */
    static constexpr const double max_motor_rpm = 3600;


    static constexpr const double max_voltage = 12.8;

/**
 * @publicsection
 */
public:

    /**
     * @public device
     * @brief VEX motor device
     * @note This is the actual underlying VEX motor device that this class wraps around and abstracts.
     */
    vex::motor* device;

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
     * @brief External gear ratio of the motor
     * @note The external ratio is the ratio of output spin to input spin.
     * For instance, if the motor spins 6 times for every 1 turn of the wheel, the external ratio should be set
     * to 1/6 as the external wheel turns 1/6 times for every 1 turn of the motor.
     */
    double external_ratio;

    /**
     * @private wheel_radius
     * @brief Radius of the wheel
     * @note The wheel radius is the distance from the center of the wheel to its outer edge, in inches.
     * The official VEX radius is not the exact radius of the wheel; it's recommended to measure the radius of the wheel with a caliper.
     */
    double wheel_radius;

    /**
     * @private spin_to_impl_variables
     * @brief Variables for asynchronous spin_to implementation
     */
    double spin_to_impl_position, spin_to_impl_tolerance, spin_to_impl_maximum, spin_to_impl_minimum, spin_to_impl_activation_ratio, spin_to_impl_integral_ratio;

    /**
     * @private gear_ratio
     * @brief Get the gear ratio of the motor cartridge
     */
    double gear_ratio(vex::gearSetting gears);

    /**
     * @private spin_to_impl
     * @brief Spin to a specified position with specified parameters
     * @param instance Pointer to the Motor instance
     * @note This function is used for asynchronous spin_to implementation. It is a helper function called by the spin_to method
     * that can run in a thread.
     * @relatesalso spin_to
     */
    static void spin_to_impl(void* instance);

/**
 * @publicsection
 */
public:

    /**
     * @public constructor
     * @brief Constructor for the Motor class
     * @param device Pointer to the VEX motor device
     * @param external_ratio External gear ratio of the motor
     * @param wheel_radius Radius of the wheel
     * @note This constructor is used to create a Motor object with a specified VEX motor device, external ratio, and wheel radius.
     */
    explicit Motor(vex::motor* device, double external_ratio, double wheel_radius);

    /**
     * @public constructor
     * @brief Constructor for the Motor class
     * @param index Port number of the motor
     * @param gears Gear setting of the motor cartridge
     * @param reverse Reverse direction of the motor
     * @param external_ratio External gear ratio of the motor
     * @param wheel_radius Radius of the wheel
     * @note This constructor is used to create a Motor object with a specified port number, gear setting, reverse direction, external ratio, and wheel radius.
     */
    explicit Motor(int32_t index, vex::gearSetting gears, bool reverse, double external_ratio, double wheel_radius);

    /**
     * @public constructor
     * @brief Copy constructor for the Motor class
     */
    explicit Motor(const Motor&);

    /**
     * @public constructor
     * @brief Move constructor for the Motor class
     */
    explicit Motor(Motor&&);

    /**
     * @public assignment operator
     * @brief Assignment operator for the Motor class
     * @param other Motor object to assign from
     * @returns A reference to the current object
     */
    Motor& operator=(const Motor&);

    /**
     * @public assignment operator
     * @brief Move assignment operator for the Motor class
     * @param other Motor object to assign from
     * @returns A reference to the current object
     */
    Motor& operator=(Motor&&);

    /**
     * @public destructor
     * @brief Destructor for the Motor class
     * @note This destructor is used to clean up the resources used by the Motor object.
     */
    ~Motor();

    /**
     * @public spin
     * @brief Spin the motor at a specified velocity
     * @param velocity Velocity to spin the motor at (in/s external tangential velocity)
     * @note This method is used to spin the motor at a specified velocity.
     */
    void spin(double velocity);

    /**
     * @public spin
     * @brief Spin the motor at a specified power
     * @param power Power to spin the motor at (percentage of maximum power)
     * @param units Units of the power (percentage or voltage)
     * @note This method is used to spin the motor at a specified power.
     */
    void spin(double power, vex::percentUnits units);

    /**
     * @public spin
     * @brief Spin the motor at a specified voltage
     * @param voltage Voltage to spin the motor at (volts)
     * @param units Units of the voltage (volts)
     * @note This method is used to spin the motor at a specified voltage.
     */
    void spin(double voltage, vex::voltageUnits units);

    /**
     * @public reset_position
     * @brief Reset the position of the motor
     * @note This method is used to reset the position of the motor to zero.
     */
    void reset_position();

    /**
     * @public stop
     * @brief Stop the motor
     */
    void stop();

    /**
     * @public connected
     * @brief Check if the motor is connected
     * @returns True if the motor is connected, false otherwise
     */
    bool connected();

    /**
     * @public device_position
     * @brief Get the raw position of the motor in specified units
     * @param units Units of the position (degrees, radians, etc.)
     * @returns The raw position of the motor in the specified units
     */
    double device_position(vex::rotationUnits units) const;

    /**
     * @public wheel_position
     * @brief Get the wheel position of the motor in specified units
     * @param units Units of the position (degrees, radians, etc.)
     * @returns The wheel position of the motor in the specified units
     */
    double wheel_position(vex::rotationUnits units) const;

    /**
     * @public max_wheel_velocity
     * @brief Get the maximum wheel tangential velocity of the motor
     * @returns The maximum wheel tangential velocity of the motor
     */
    double max_wheel_velocity() const;

    /**
     * @public spin_to
     * @brief Spin the motor to a specified position with specified parameters
     * @param position Position to spin to (degrees)
     * @param tolerance Tolerance for the spin (degrees)
     * @param maximum Maximum voltage for the motors (volts)
     * @param minimum Minimum voltage for the motors (volts)
     * @param activation_ratio Activation ratio for the PID controller
     * @param integral_ratio Integral ratio for the PID controller
     * @param block Block until the spin is complete- true to block, false to run asynchronously. Defaults to true.
     * @attention
     * Activation and integral ratios are given in the range of [0.0, 1.0]. Since the PID controller
     * expects an absolute error, the activation ratio is used to determine when to activate the controller. For
     * example, with a motor turn of 90 degrees and an activation_ratio of 0.3, the controller will output maximum
     * voltage until the robot is within 90*0.3=27 degrees of the target angle. Similarly, if the integral_ratio is
     * set to 0.2, the controller will disable integration until the wheel is within 90*0.2=18 degrees of the target position.
     */
    void spin_to(double position, double tolerance = 2.0, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2, bool block = true);
};

#endif // #ifndef MOTOR_HPP