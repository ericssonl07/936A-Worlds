#ifndef PID_HPP
#define PID_HPP

#include <utility>
#include <map>
#include <string>

class PID {
public:
    // Default constructor with some reasonable defaults.
    PID();

    // Parameterized constructor.
    PID(double kp, double ki, double kd, double target,
        double tolerance, double max_value, double min_value,
        double activation_threshold, double integration_threshold,
        double derivative_threshold, double max_integral, double gamma,
        bool deactivate_integral_on_error_sign = true);

    // Reset the controller (integral and last error).
    void reset();

    // Get/set target.
    const double& target() const;
    double& target(double new_target);

    // Get/set the output limits as a pair (first = min_value, second = max_value).
    const std::pair<double, double>& limits() const;
    std::pair<double, double>& limits(double new_min, double new_max);

    // Individual getters.
    const double& max_value() const;
    const double& min_value() const;
    const double& activation_threshold() const;
    const double& integration_threshold() const;
    const double& derivative_threshold() const;
    const double& max_integral() const;
    const double& gamma() const;
    const double& tolerance() const;

    // Compute the PID output for the given current value.
    // If 'limit' is true, the output is clamped and modified based on the special thresholds.
    double calculate(double current, bool limit = true);

    // Returns whether the controller considers itself "arrived" (|error| < tolerance and |derivative| < derivative_threshold).
    bool arrived() const;

    // Set the PID gains.
    void set_gains(double kp, double ki, double kd);

    // Get P, I, D, and output.
    std::map<std::string, double> get_info();

private:
    // Gains.
    double kp_;
    double ki_;
    double kd_;

    // Target value.
    double target_;

    // Output value.
    double output_;

    // For storing the previous error.
    double last_error_;

    // Accumulated integral.
    double integral_;

    // Flag that is true when the controller is considered to have arrived at target.
    bool arrived_;

    // Special private members for additional functionality.
    double tolerance_;              // Allowed error.
    double max_value_;              // Upper bound on |output|.
    double min_value_;              // Lower bound on |output| (if nonzero).
    double activation_threshold_;   // When |error| exceeds this, output is max_value_ with proper sign. "P_threshold"
    double integration_threshold_;  // When |error| is above this, integral is not accumulated. "I_threshold"
    double derivative_threshold_;   // When |derivative| exceeds this, the controller is not considered to have arrived. "D_threshold"
    double max_integral_;           // Maximum absolute value for the integral term. "I_max"
    double gamma_;                  // Factor to decay (or “leak”) the integral term each timestep.
    bool deactivate_integral_on_error_sign_; // Reset integral when error sign changes.

    // Cached pair for the limits getter (first = min_value, second = max_value).
    std::pair<double, double> limits_pair_;
};

#endif // PID_HPP