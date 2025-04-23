#include <pid.hpp>
#include <cmath>
#include <cstdio>
#include <definitions.hpp>

PID::PID()
    : kp_(0.0), ki_(0.0), kd_(0.0),
      target_(0.0),
      last_error_(0.0),
      integral_(0.0),
      arrived_(false),
      tolerance_(0.01),
      max_value_(1.0),
      min_value_(0.0),
      activation_threshold_(1.0),
      integration_threshold_(1.0),
      derivative_threshold_(0.1),
      max_integral_(10.0),
      gamma_(0.9),
      deactivate_integral_on_error_sign_(true),
      limits_pair_(std::make_pair(min_value_, max_value_))
{
}

// Parameterized constructor.
PID::PID(double kp, double ki, double kd, double target,
         double tolerance, double max_value, double min_value,
         double activation_threshold, double integration_threshold,
         double derivative_threshold, double max_integral, double gamma,
         bool deactivate_integral_on_error_sign)
    : kp_(kp), ki_(ki), kd_(kd),
      target_(target),
      last_error_(0.0),
      integral_(0.0),
      arrived_(false),
      tolerance_(tolerance),
      max_value_(max_value),
      min_value_(min_value),
      activation_threshold_(activation_threshold),
      integration_threshold_(integration_threshold),
      derivative_threshold_(derivative_threshold),
      max_integral_(max_integral),
      gamma_(gamma),
      deactivate_integral_on_error_sign_(deactivate_integral_on_error_sign),
      limits_pair_(std::make_pair(min_value_, max_value_))
{
}

// Reset the integral and last error.
void PID::reset() {
    last_error_ = 0.0;
    integral_ = 0.0;
    arrived_ = false;
}

// Get the target value.
const double& PID::target() const {
    return target_;
}

// Set the target value.
double& PID::target(double new_target) {
    target_ = new_target;
    return target_;
}

// Get the limits (min_value and max_value).
const std::pair<double, double>& PID::limits() const {
    return limits_pair_;
}

// Set the limits and update the cached pair.
std::pair<double, double>& PID::limits(double new_min, double new_max) {
    min_value_ = new_min;
    max_value_ = new_max;
    limits_pair_.first = min_value_;
    limits_pair_.second = max_value_;
    return limits_pair_;
}

// Getter for max_value.
const double& PID::max_value() const {
    return max_value_;
}

// Getter for min_value.
const double& PID::min_value() const {
    return min_value_;
}

// Getter for activation_threshold.
const double& PID::activation_threshold() const {
    return activation_threshold_;
}

// Getter for integration_threshold.
const double& PID::integration_threshold() const {
    return integration_threshold_;
}

// Getter for derivative_threshold.
const double& PID::derivative_threshold() const {
    return derivative_threshold_;
}

// Getter for max_integral.
const double& PID::max_integral() const {
    return max_integral_;
}

// Getter for gamma.
const double& PID::gamma() const {
    return gamma_;
}

// Getter for tolerance.
const double& PID::tolerance() const {
    return tolerance_;
}

// Compute the PID output for the given current value.
double PID::calculate(double current, bool limit) {
    // Calculate error.
    double error = target_ - current;

    // Compute derivative (assuming constant time steps).
    double derivative = error - last_error_;

    // Deactivate the integral if:
    // - Error sign has changed and integral deactivation is enabled.
    // - Error is outside the integration threshold.
    // - Error is within the tolerance range (stop accumulating- close to target).
    if (limit and ((deactivate_integral_on_error_sign_ && (error * last_error_ < 0)) or (std::fabs(error) > integration_threshold_) or (std::fabs(error) < tolerance_))) {
        integral_ = 0.0;
    } else {
        // Decay the integral term and then add the new error.
        integral_ = integral_ * gamma_ + error;
        // Clamp the integral to avoid windup.
        if (std::fabs(integral_) > max_integral_) {
            integral_ = sgn(integral_) * max_integral_;
        }
    }

    // Compute the raw PID output.
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    if (limit) {
        // If the error is large, use the activation threshold to immediately set max output.
        if (std::fabs(error) > activation_threshold_) {
            output = sgn(error) * max_value_;
        } else {
            // Ensure that the output’s magnitude is between min_value_ and max_value_.
            double abs_output = std::fabs(output);
            if (abs_output < min_value_ && abs_output > 0) {
                output = sgn(output) * min_value_;
            } else if (abs_output > max_value_) {
                output = sgn(output) * max_value_;
            }
        }
    }

    // Update last error.
    last_error_ = error;

    // Determine if the controller has "arrived".
    // Arrived is true if both the absolute error is within tolerance and the derivative is within derivative_threshold.
    arrived_ = std::fabs(error) < tolerance_ && std::fabs(derivative) < derivative_threshold_;

    return output_ = output;
}

double PID::calculate_raw(double current, bool limit) {
    // Calculate error
    double error = target_ - current;
    
    // Update integral term (standard accumulation without decay)
    integral_ += error;
    
    // Apply anti-windup by clamping the integral
    if (limit && std::fabs(integral_) > max_integral_) {
        integral_ = sgn(integral_) * max_integral_;
    }
    
    // Calculate derivative term
    double derivative = error - last_error_;
    
    // Compute the PID output
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    
    // Apply simple output limiting if needed
    if (limit && std::fabs(output) > max_value_) {
        output = sgn(output) * max_value_;
    }
    
    // Update last error for next iteration
    last_error_ = error;
    
    return output_ = output;
}

// Return whether the controller is considered to have arrived.
bool PID::arrived() const {
    return arrived_;
}

// Set the PID gains.
void PID::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

// Get P, I, D, and output.
std::map<std::string, double> PID::get_info() {
    return {
        {"P", kp_},
        {"I", ki_},
        {"D", kd_},
        {"Output", output_}
    };
}