#ifndef PID_HPP
#define PID_HPP

#include <utility>
#include <map>
#include <string>

/**
 * @class PID
 * @file pid.hpp
 * @author @ericssonl07
 * @date 2025-04-20
 * @brief
 * This class implements a customized PID (Proportional-Integral-Derivative) controller for controlling a system.
 * @note
 * The PID controller is a control loop feedback mechanism widely used in industrial control systems.
 * It calculates an "error" value as the difference between a desired setpoint and a measured process variable.
 * The controller attempts to minimize the error by adjusting the process control inputs.
 * The PID controller consists of three namesake components:
 * - Proportional: The proportional term produces an output value that is proportional to the current error value.
 * - Integral: The integral term is concerned with the accumulation of past errors. If the error has been present
 * for a long time, the integral term will increase, leading to a larger output.
 * - Derivative: The derivative term is a prediction of future error based on its rate of change. The greater the
 * present rate of change, the more the derivative term will resist the change, preventing overshoot.
 * 
 * The traditional PID controller suffers from a few issues:
 * - Integral windup: When the controller is saturated, the integral term can accumulate a large error, leading to overshoot.
 * - Termination conditions: The controller may not be able to determine when it has arrived at the target.
 * - Inefficiency at startup: The controller is usually not very efficient at startup, and a bang-bang controller is often better initially.
 * 
 * This implementation of the PID controller includes several features to address these issues (mainly integral windup):
 * - Activation threshold: The controller activates below a certain error threshold; above this threshold, the output is set to the maximum value.
 * - Integration threshold: The integral term is not accumulated when the error is above this threshold. Addresses integral windup. In the
 * Wikipedia article (see below), this is referred to as "conditional integration."
 * - Derivative threshold: The controller is not considered to have arrived when the derivative exceeds this threshold.
 * Addresses termination conditions- a system that has small error but large derivative is likely "sweeping" past the target and not actually arrived.
 * - Maximum integral: The integral term is limited to a maximum value. Addresses integral windup.
 * - Gamma: The integral term is decayed by a factor of gamma each timestep so it "forgets" past errors gradually. Addresses integral windup.
 * - Deactivate integral on error sign change: The integral term is reset when the error sign changes. Thus when the system sweeps past the target,
 * the integral term is reset to zero so it doesn't keep pushing the system in the wrong direction. Addresses integral windup. In the Wikipedia article,
 * this is referred to as "Clegg integration."
 * @example
 * ```cpp
 * // Example usage of the PID class:
 * PID pid(1.0, 0.1, 0.01, 0.0, 0.05, 12.0, -12.0, 1.0, 0.5, 0.5, 1.0, 50.0, 0.99);
 * double output = pid.calculate(current_value);
 * bool arrived = pid.arrived();
 * pid.reset(); // Reset the controller
 * ```
 * @see
 * [Integral Windup Solutions](https://en.wikipedia.org/wiki/Integral_windup#Solutions)
 */
class PID {
/**
 * @publicsection
 */
public:

    /**
     * @public constructor
     * @brief Default constructor for PID
     */
    PID();

    /**
     * @public constructor
     * @brief Constructor for PID
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param target Target value
     * @param tolerance Allowed error
     * @param max_value Upper bound on |output|
     * @param min_value Lower bound on |output| (if nonzero)
     * @param activation_threshold When |error| exceeds this, output is max_value_ with proper sign. Also known as "P_threshold"
     * @param integration_threshold When |error| is above this, integral is not accumulated. Also known as "I_threshold"
     * @param derivative_threshold When |derivative| exceeds this, the controller is not considered to have arrived. Also known as "D_threshold"
     * @param max_integral Maximum absolute value for the integral term. Also known as "I_max"
     * @param gamma Factor to decay (or “leak”) the integral term each timestep.
     * @param deactivate_integral_on_error_sign Reset integral when error sign changes (see Clegg integrator).
     * @see
     * [Clegg Integrator](https://en.wikipedia.org/wiki/Integral_windup#Solutions)
     * @attention All parameters are expected to be positive.
     */
    PID(double kp, double ki, double kd, double target,
        double tolerance, double max_value, double min_value,
        double activation_threshold, double integration_threshold,
        double derivative_threshold, double max_integral, double gamma,
        bool deactivate_integral_on_error_sign = true);

    /**
     * @public reset
     * @brief Reset the PID controller
     * @details This method resets the PID controller to its initial state with integral set to 0.
     */
    void reset();

    /**
     * @public target
     * @brief Get the target value
     * @returns The target value as a const reference.
     */
    const double& target() const;

    /**
     * @public target
     * @brief Set the target value
     * @param new_target The new target value.
     * @returns A non-const reference to the newly set target value.
     */
    double& target(double new_target);

    /**
     * @public limits
     * @brief Get the limits of the PID controller
     * @returns A const reference to the pair of doubles representing the minimum and maximum limits.
     */
    const std::pair<double, double>& limits() const;

    /**
     * @public limits
     * @brief Set the limits of the PID controller
     * @param new_min The new minimum limit.
     * @param new_max The new maximum limit.
     * @returns A non-const reference to the pair of doubles representing the limits.
     */
    std::pair<double, double>& limits(double new_min, double new_max);

    /**
     * @public max_value
     * @brief Get the maximum value
     * @returns The maximum value as a const reference.
     */
    const double& max_value() const;

    /**
     * @public min_value
     * @brief Get the minimum value
     * @returns The minimum value as a const reference.
     */
    const double& min_value() const;

    /**
     * @public activation_threshold
     * @brief Get the activation threshold
     * @returns The activation threshold as a const reference.
     */
    const double& activation_threshold() const;

    /**
     * @public integration_threshold
     * @brief Get the integration threshold
     * @returns The integration threshold as a const reference.
     */
    const double& integration_threshold() const;

    /**
     * @public derivative_threshold
     * @brief Get the derivative threshold
     * @returns The derivative threshold as a const reference.
     */
    const double& derivative_threshold() const;

    /**
     * @public max_integral
     * @brief Get the maximum integral value
     * @returns The maximum integral value as a const reference.
     */
    const double& max_integral() const;

    /**
     * @public gamma
     * @brief Get the gamma value
     * @returns The gamma value as a const reference.
     */
    const double& gamma() const;

    /**
     * @public deactivate_integral_on_error_sign
     * @brief Get the deactivate integral on error sign flag
     * @returns The deactivate integral on error sign flag as a const reference.
     */
    const double& tolerance() const;

    /**
     * @public calculate
     * @brief Compute the PID output for the given current value
     * @param current The current value.
     * @param limit Whether to apply custom (non-traditional) PID calculations.
     * @details
     * This method computes the PID output for the given current value.
     * 
     * First, it calculates the error as the signed difference between the target and current values.
     * 
     * If limit is false, then the output is calculated using the traditional PID formula:
     * 
     * output = kp_ * error + ki_ * integral_ + kd_ * derivative_
     * 
     * If limit is true, then the output is calculated using the custom PID procedure:
     * 
     * - If |error| > activation_threshold, output = max_value_ * sign(error).
     * - If |error| < activation_threshold, output = kp_ * error.
     * - If |error| < integration_threshold, output += ki_ * integral_.
     * - output += kd_ * derivative_.
     * - If |output| > max_value_, output = max_value_ * sign(output).
     * - If |output| < min_value_, output = min_value_ * sign(output).
     * 
     * Termination conditions are determined by the following:
     * 
     * arrived_ = (|error| < tolerance_ && |derivative| < derivative_threshold_).
     * 
     */
    double calculate(double current, bool limit = true);

    double calculate_raw(double current, bool limit);

    /**
     * @public arrived
     * @brief Check if the controller has arrived at the target
     * @returns True if the controller has arrived at the target, false otherwise.
     * @details
     * This method checks if the controller has arrived at the target.
     * 
     * arrived = (|error| < tolerance_ && |derivative| < derivative_threshold_).
     */
    bool arrived() const;

    /**
     * @public set_gains
     * @brief Set the PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void set_gains(double kp, double ki, double kd);

    /**
     * @public get_info
     * @brief Get information about the PID controller
     * @details
     * This method returns a map containing information about the PID controller:
     * 
     * {"P": kp_, "I": ki_, "D": kd_, "Output": output_}
     * 
     * @note
     * This method returns a map containing information about the PID controller:
     * 
     * {"P": kp_, "I": ki_, "D": kd_, "Output": output_}
     * 
     * @returns A map containing information about the PID controller: {"P": kp_, "I": ki_, "D": kd_, "Output": output_}
     */
    std::map<std::string, double> get_info();

/**
 * @privatesection
 */
private:

    /**
     * @private kp_
     * @brief Proportional gain
     */
    double kp_;

    /**
     * @private ki_
     * @brief Integral gain
     */
    double ki_;

    /**
     * @private kd_
     * @brief Derivative gain
     */
    double kd_;

    /**
     * @private target_
     * @brief Target value
     */
    double target_;

    /**
     * @private output_
     * @brief Output value
     */
    double output_;

    /**
     * @private last_error_
     * @brief Last error value
     */
    double last_error_;

    /**
     * @private integral_
     * @brief Integral value
     */
    double integral_;

    /**
     * @private arrived_
     * @brief Flag indicating if the controller has arrived at the target
     * @details
     * arrived_ = (|error| < tolerance_ && |derivative| < derivative_threshold_).
     */
    bool arrived_;

    /**
     * @private tolerance_
     * @brief Allowed error
     */
    double tolerance_;

    /**
     * @private max_value_
     * @brief Upper bound on |output|
     */
    double max_value_;

    /**
     * @private min_value_
     * @brief Lower bound on |output| (if nonzero)
     */
    double min_value_;

    /**
     * @private activation_threshold_
     * @brief Activation threshold
     * @details
     * When |error| exceeds this, output is max_value_ with proper sign. Also known as "P_threshold"
     */
    double activation_threshold_;

    /**
     * @private integration_threshold_
     * @brief Integration threshold
     * @details
     * When |error| is above this, integral is not accumulated. Also known as "I_threshold"
     */
    double integration_threshold_;

    /**
     * @private derivative_threshold_
     * @brief Derivative threshold
     * @details
     * When |derivative| exceeds this, the controller is not considered to have arrived. Also known as "D_threshold"
     */
    double derivative_threshold_;

    /**
     * @private max_integral_
     * @brief Maximum absolute value for the integral term. Also known as "I_max"
     */
    double max_integral_;

    /**
     * @private gamma_
     * @brief Factor to decay (or “leak”) the integral term each timestep.
     */
    double gamma_;

    /**
     * @private deactivate_integral_on_error_sign_
     * @brief Flag for whether or not to use Clegg integration.
     */
    bool deactivate_integral_on_error_sign_;

    /**
     * @private limits_pair_
     * @brief Cached pair of doubles representing the limits (min_value and max_value).
     */
    std::pair<double, double> limits_pair_;
};

#endif // PID_HPP