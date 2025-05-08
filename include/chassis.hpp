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

/**
 * @class Chassis
 * @file chassis.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This class represents a basic chassis for a robot, providing methods for controlling its movement and tracking its position.
 * @details
 * The Chassis class is designed to work with a set of motors, an inertial sensor, and a controller.
 * It allows for precise control of the robot's movement, including turning and following paths.
 * The class also provides methods for resetting the robot's position and stopping its movement.
 * The constructor initializes the chassis with the specified parameters, including motor groups, tracking wheels, and odometry settings.
 * The class uses a PID controller for turning and following paths, allowing for smooth and accurate movement.
 * The class also includes methods for controlling the robot using different control schemes, such as arcade and tank control.
 * The class is designed to be used in a VEX robotics environment, utilizing the VEX C++ API for motor control and sensor readings.
 * The class is designed to be flexible and extensible, allowing for customization and adaptation to different robot designs and control strategies.
 * Position tracking is achieved using odometry, which calculates the robot's position based on the movement of its wheels and the readings from the inertial sensor.
 * @example
 * ```cpp
 * // Example usage of the Chassis class:
 * // Assuming you have already created instances of MotorGroup, vex::rotation, vex::inertial, and vex::controller, as well as defined the necessary parameters:
 * Chassis base(&lmg, &rmg, &left_track, &back_track, &imu, &controller, base_width, left_offset, back_offset, wheel_radius, tracking_radius, external_ratio, max_radial_acceleration);
 * base.set_pose(0, 0, 0); // Set initial position
 * base.forward(12, 0.05); // Move forward 12 inches with a tolerance of 0.05in
 * base.turn_to(M_PI / 2, 0.05); // Turn to 90 degrees (absolute) with a tolerance of 0.05rad
 * base.turn(M_PI / 2, 0.05); // Turn to 90 degrees (relative) with a tolerance of 0.05rad
 * base.follow_path(path, 2.5, 9.0); // Follow a predefined path with specified tolerance and lookahead distance
 * base.drift_in_place(12.0, M_PI, 1.0); // Drift in place to a new heading with specified strength
 * base.drift_after_distance(10, 12.0, M_PI, 1.0); // Drift after moving a certain distance
 * base.corner_reset(12.0); // Reset the robot's position to a corner with specified offset
 * base.reset_position(); // Reset the robot's position to (0, 0, 0)
 * base.steer(6.0, 6.0); // Steer the robot with specified left and right voltages
 * base.forward_timer(2.0, 12.0, 0.5); // Move forward for a specified time with a base voltage and corrective strength
 * base.steer_timer(6.0, 6.0, 2.0); // Steer for a specified time with specified left and right voltages
 * base.stop(); // Stop the robot
 * ```
 * @see
 * Odometry
 * 
 * MotorGroup
 * 
 * Path
 * 
 * Pursuit
 * 
 * PID
 */
class Chassis {

/**
 * @protectedsection
 */
protected:

    /**
     * @private left
     * @brief Left motor group
     */
    MotorGroup* left;

    /**
     * @private right
     * @brief Right motor group
     */
    MotorGroup* right;

    /**
     * @private forward_track
     * @brief Forward/backward direction tracking wheel
     */
    vex::rotation* forward_track;

    /**
     * @private side_track
     * @brief Left/right direction tracking wheel
     */
    vex::rotation* side_track;

    /**
     * @private imu
     * @brief Inertial sensor
     * @attention The imu is used for tracking the robot's rotation and position. It must be calibrated before use (see example code).
     * Additionally, the imu must follow mathematical conventions for rotation, where positive rotation is counter-clockwise and negative rotation is clockwise.
     * The imu should be mounted so that its yaw axis is aligned with the vertical axis of the game field.
     * @example
     * // Example code for calibrating the imu:
     * 
     * imu.calibrate();
     * 
     * while (imu.is_calibrating()) {
     * 
     *   vex::this_thread::sleep_for(100);
     * 
     * }
     * 
     * // After calibration, the imu is ready to be used for tracking the robot's rotation and position.
     */
    vex::inertial* imu;

    /**
     * @private controller
     * @brief Controller for user input
     */
    vex::controller* controller;

    /**
     * @private odometry
     * @brief Odometry object for tracking the robot's position
     * @see Odometry
     */
    Odometry* odometry;

    /**
     * @private base_width
     * @brief Width of the robot's base
     * @attention The base width is the distance between the left and right motor group wheels, in inches.
     */
    double base_width;

    /**
     * @private forward_offset
     * @brief Offset of the front/back tracking wheel (parallel to the robot's velocity vector) to the tracking center.
     * @attention Left is positive, right is negative. Given in inches.
     */
    double forward_offset;

    /**
     * @private side_offset
     * @brief Offset of the left/right tracking wheel (perpendicular to the robot's velocity vector) to the tracking center.
     * @attention Forward is negative, backward is positive. Given in inches.
     */
    double side_offset;

    /**
     * @private wheel_radius
     * @brief Radius of the robot's wheels
     * @attention The wheel radius is the distance from the center of the wheel to its outer edge, in inches.
     * The official VEX radius is not the exact radius of the wheel; it's recommended to measure the radius of the wheel with a caliper.
     */
    double wheel_radius;

    /**
     * @private tracking_radius
     * @brief Radius of the tracking wheels
     * @attention The tracking radius is the distance from the center of the tracking wheel to its outer edge, in inches.
     * The official VEX radius is not the exact radius of the wheel; it's recommended to measure the radius of the wheel with a caliper.
     */
    double tracking_radius;

    /**
     * @private external_ratio
     * @brief External gear ratio of the robot's motors
     * @attention The external gear ratio is the ratio of the number of teeth on the driven gear to the number of teeth on the driving gear.
     * This value is used to calculate the speed and torque of the motors. The gear ratio is given as the ratio of output spin to input spin.
     * For instance, if the motor spins 6 times for every 1 turn of the wheel, the external ratio should be set to 1/6 as the external
     * wheel turns 1/6 times for every 1 turn of the shaft. There is no need to account for the cartridge, which vex::motor does automatically.
     */
    double external_ratio;

    /**
     * @private max_radial_acceleration
     * @brief Maximum radial acceleration of the robot
     * @attention The maximum radial acceleration is the maximum rate at which the robot can change its direction of motion, in inches per second squared.
     * This value is used to calculate the maximum speed and acceleration of the robot when following a path.
     */
    double max_radial_acceleration;

/**
 * @publicsection
 */
public:

    /**
     * @public constructor
     * @brief Constructor for the Chassis class
     * @param left Left motor group
     * @param right Right motor group
     * @param forward_track Forward tracking wheel (parallel to the robot's velocity vector)
     * @param side_track Side tracking wheel (perpendicular to the robot's velocity vector)
     * @param imu Inertial sensor (calibrated and with vex::turnType::left specifier)
     * @param controller Controller for user input
     * @param base_width Width of the robot's base (distance between left and right motor group wheels)
     * @param forward_offset Offset of the front/back tracking wheel to the tracking center (left is positive, right is negative)
     * @param side_offset Offset of the left/right tracking wheel to the tracking center (forward is negative, backward is positive)
     * @param wheel_radius Radius of the robot's wheels (distance from the center of the wheel to its outer edge)
     * @param tracking_radius Radius of the tracking wheels (distance from the center of the tracking wheel to its outer edge)
     * @param external_ratio External gear ratio of the robot's motors (ratio of output spin to input spin)
     * @param max_radial_acceleration Maximum radial acceleration of the robot (maximum rate at which the robot can change its direction of motion)
     */
    Chassis(MotorGroup* left, MotorGroup* right, vex::rotation* forward_track, vex::rotation* side_track,
            vex::inertial* imu, vex::controller* controller, double base_width, double forward_offset,
            double side_offset, double wheel_radius, double tracking_radius, double external_ratio, double max_radial_acceleration);

    /**
     * @public steer
     * @brief Steer the robot with specified left and right voltages
     * @param left_voltage Voltage for the left motor group
     * @param right_voltage Voltage for the right motor group
     * @details The steer method allows for precise control of the robot's movement by applying different voltages to the left and right motor groups.
     * @note The function respects the maximum voltage of the motors, which is 12V. Voltages greater than 12V will be capped to 12V by proportionally
     * reducing the voltages of both motors.
     */
    void steer(double left_voltage, double right_voltage);

    /**
     * @public set_pose
     * @brief Set the robot's position and rotation
     * @param x X-coordinate of the robot's position (inches)
     * @param y Y-coordinate of the robot's position (inches)
     * @param rotation Rotation of the robot (radians)
     * @details The set_pose method allows for setting the robot's position and rotation in the odometry system.
     * This is useful for resetting the robot's position after a movement or when starting a new task.
     */
    void set_pose(double x, double y, double rotation);
    
    /**
     * @public follow_path
     * @brief Follow a predefined path with specified tolerance and lookahead distance
     * @param path Path to follow
     * @param tolerance Tolerance for following the path (inches)
     * @param lookahead Lookahead distance for the path following (inches)
     * @details The follow_path method allows the robot to follow a predefined path using a pursuit algorithm.
     * The method takes into account the robot's current position and rotation, as well as the specified tolerance and lookahead distance.
     */
    void follow_path(Path path, double tolerance, double lookahead); // Tested on Friday, April 11th, 2025

    /**
     * @public turn_to
     * @brief Turn the robot to a specified angle with specified tolerance
     * @param angle Angle to turn to (radians)
     * @param tolerance Tolerance for the turn (radians)
     * @param maximum Maximum voltage for the motors (volts)
     * @param minimum Minimum voltage for the motors (volts)
     * @param activation_ratio Activation ratio for the PID controller
     * @param integral_ratio Integral ratio for the PID controller
     * @details The turn_to method allows the robot to turn to a specified angle using a PID controller.
     * The method takes into account the robot's current rotation and the specified tolerance.
     * @attention
     * Activation and integral ratios are given in the range of [0.0, 1.0]. Since the PID controller
     * expects an absolute error, the activation ratio is used to determine when to activate the controller. For
     * example, with a turn of 90 degrees and an activation_ratio of 0.3, the controller will output maximum
     * voltage until the robot is within 90*0.3=27 degrees of the target angle. Similarly, if the integral_ratio is
     * set to 0.2, the controller will disable integration until the robot is within 90*0.2=18 degrees of the target angle.
     * 
     * The turn_to method accounts for coterminality, and it turns to an absolute angle.
     */
    void turn_to(double angle, double tolerance, double maximum = 12.0, double minimum = 0.02, double activation_threshold = M_PI / 4, double integral_threshold = M_PI / 10, double derivative_threshold = M_PI / 200, double p = 10.0, double i = 0.3, double d = 15.0); // Tested on Friday, April 11th, 2025

    /**
     * @public turn
     * @brief Turn the robot by a specified angle with specified tolerance
     * @param angle Angle to turn by (radians)
     * @param tolerance Tolerance for the turn (radians)
     * @param maximum Maximum voltage for the motors (volts)
     * @param minimum Minimum voltage for the motors (volts)
     * @param activation_ratio Activation ratio for the PID controller
     * @param integral_ratio Integral ratio for the PID controller
     * @details The turn method allows the robot to turn by a specified angle using a PID controller.
     * The method takes into account the robot's current rotation and the specified tolerance.
     * @attention
     * Activation and integral ratios are given in the range of [0.0, 1.0]. Since the PID controller
     * expects an absolute error, the activation ratio is used to determine when to activate the controller. For
     * example, with a turn of 90 degrees and an activation_ratio of 0.3, the controller will output maximum
     * voltage until the robot is within 90*0.3=27 degrees of the target angle. Similarly, if the integral_ratio is
     * set to 0.2, the controller will disable integration until the robot is within 90*0.2=18 degrees of the target angle.
     * 
     * The turn method does NOT account for coterminality, and it turns to a relative angle.
     */
    void turn(double angle, double tolerance, double maximum = 12.0, double minimum = 0.02, double activation_threshold = M_PI / 4, double integral_threshold = M_PI / 10, double derivative_threshold = M_PI / 200, double p = 10.0, double i = 0.3, double d = 15.0);

    /**
     * @public forward
     * @brief Move the robot forward by a specified distance with specified tolerance
     * @param distance Distance to move forward (inches)
     * @param tolerance Tolerance for the movement (inches)
     * @param maximum Maximum voltage for the motors (volts)
     * @param minimum Minimum voltage for the motors (volts)
     * @param activation_ratio Activation ratio for the PID controller
     * @param integral_ratio Integral ratio for the PID controller
     * @param p Proportional gain for the PID controller
     * @param d Derivative gain for the PID controller
     * @param i Integral gain for the PID controller
     * @details The forward method allows the robot to move forward by a specified distance using a PID controller.
     * The method takes into account the robot's current position and rotation, as well as the specified tolerance.
     * @attention
     * Activation and integral ratios are given in the range of [0.0, 1.0]. Since the PID controller
     * expects an absolute error, the activation ratio is used to determine when to activate the controller. For
     * example, with a distance of 12 inches and an activation_ratio of 0.3, the controller will output maximum
     * voltage until the robot is within 12*0.3=3.6 inches of the target distance. Similarly, if the integral_ratio is
     * set to 0.2, the controller will disable integration until the robot is within 12*0.2=2.4 inches of the target distance.
     */
    void forward(double distance, double tolerance = 0.05, double maximum = 12.0, double minimum = 0.02, double activation_threshold = 24.0, double integral_threshold = 6.0, double derivative_threshold = 0.2, double p = 0.50, double i = 0.08, double d = 3.0); // Tested on Friday, April 11th, 2025

    /**
     * @public forward_timer
     * @brief Move the robot forward for a specified time with specified base voltage and corrective strength
     * @param time Time to move forward (seconds)
     * @param base_voltage Base voltage for the motors (volts)
     * @param corrective_strength Corrective strength for the movement
     * @details The forward_timer method allows the robot to move forward for a specified time using a PID controller.
     * The method takes into account the specified base voltage and corrective strength.
     * 
     * @note corrective_strength is a multiplier for the PID controller's gains. It is used to adjust the strength of the corrective action
     * applied to the robot's movement so that it can maintain a straight line while moving forward.
     */
    void forward_timer(double time, double base_voltage = 12.0, double corrective_strength = 1.0); // Tested on Friday, April 11th, 2025

    /**
     * @public steer_timer
     * @brief Steer the robot for a specified time with specified left and right voltages
     * @param left_voltage Voltage for the left motor group (volts)
     * @param right_voltage Voltage for the right motor group (volts)
     * @param time Time to steer (seconds)
     * @details The steer_timer method allows the robot to steer for a specified time using specified left and right voltages.
     * The method takes into account the specified time for steering.
     */
    void steer_timer(double left_voltage, double right_voltage, double time); // Tested on Friday, April 11th, 2025

    /**
     * @public drift_in_place
     * @brief Drift the robot in place to a new heading with specified bias and strength
     * @param bias Bias for the drift (volts)- the higher the bias, the more the robot will move forward
     * @param new_heading New heading to drift to (radians)- this is absolute (no coterminality accounted for to enable more complex drifts)
     * @param strength Strength of the drift (volts)- this is a multiplier for the PID controller's gains
     * @param maximum Maximum voltage for the motors (volts)
     * @param minimum Minimum voltage for the motors (volts)
     * @param activation_ratio Activation ratio for the PID controller
     * @param integral_ratio Integral ratio for the PID controller
     * @details The drift_in_place method allows the robot to drift in place to a new heading using a PID controller.
     * The method takes into account the specified bias, strength, and other parameters.
     * 
     * @note The drift_in_place method is designed to be used for more complex drifts, where the robot may need to move forward while turning.
     * The method uses a PID controller to adjust the robot's movement based on the specified bias and strength.
     * 
     * @attention
     * Activation and integral ratios are given in the range of [0.0, 1.0]. Since the PID controller
     * expects an absolute error, the activation ratio is used to determine when to activate the controller. For
     * example, with a turn of 90 degrees and an activation_ratio of 0.3, the controller will output maximum
     * voltage until the robot is within 90*0.3=27 degrees of the target angle. Similarly, if the integral_ratio is
     * set to 0.2, the controller will disable integration until the robot is within 90*0.2=18 degrees of the target angle.
     */
    void drift_in_place(double bias, double new_heading, double strength, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2); // Tested on Friday, April 11th, 2025

    /**
     * @public drift_after_distance
     * @brief Drift the robot after moving a certain distance to a new heading with specified bias and strength
     * @param distance Distance to move before drifting (inches)
     * @param bias Bias for the drift (volts)- the higher the bias, the more the robot will move forward
     * @param new_heading New heading to drift to (radians)- this is absolute (no coterminality accounted for to enable more complex drifts)
     * @param strength Strength of the drift (volts)- this is a multiplier for the PID controller's gains
     * @param maximum Maximum voltage for the motors (volts)
     * @param minimum Minimum voltage for the motors (volts)
     * @param activation_ratio Activation ratio for the PID controller
     * @param integral_ratio Integral ratio for the PID controller
     * @details The drift_after_distance method allows the robot to drift after moving a certain distance using a PID controller.
     * The method takes into account the specified bias, strength, and other parameters.
     * 
     * @note The drift_after_distance method is designed to be used for more complex drifts, where the robot may need to move forward before drifting.
     */
    void drift_after_distance(double distance, double bias, double new_heading, double strength, double maximum = 12.0, double minimum = 0.02, double activation_ratio = 0.3, double integral_ratio = 0.2); // Tested on Friday, April 11th, 2025

    /**
     * @public corner_reset
     * @brief Reset the robot's position to a corner with specified offset- robot must be physically aligned with the corner in a triangle.
     * @param lengthwise_offset Offset from the line of the back of the robot to the tracking center. What is the perpendicular distance
     * from the tracking center to the hypotenuse of the triangle formed by the aligning robot?
     * @details The corner_reset method allows the robot to reset its position to a corner with a specified offset.
     * This is useful for resetting the robot's position during autonomous.
     * 
     * @todo Implement a more complex corner reset method that uses distance sensors so that resets are possible with loaded clamps.
     */
    void corner_reset(double lengthwise_offset); // Tested on Friday, April 11th, 2025

    /**
     * @public reset_position
     * @brief Reset the robot's position to (0, 0, 0)
     * @details The reset_position method allows the robot to reset its position to (0, 0, 0).
     * This is useful for resetting the robot's position during autonomous.
     */

    void corner_reset_forward(double lengthwise_offset);

    /**
     * @public corner_reset_forward
     * @brief Reset the robot's position to a corner with specified offset- robot must be physically aligned with the corner in a triangle.
     * @param lengthwise_offset Offset from the line of the back of the robot to the tracking center. What is the perpendicular distance
     * from the tracking center to the hypotenuse of the triangle formed by the aligning robot?
     * @details The corner_reset_forward method allows the robot to reset its position to a corner with a specified offset.
     */

    void reset_position();

    /**
     * @public stop
     * @brief Stop the robot
     * @details The stop method allows the robot to stop its movement.
     * This is useful for stopping the robot during autonomous or when the user wants to stop the robot.
     * 
     * This function uses vex::brakeType::brake, so the robot will stop immediately and lock.
     */
    void stop();

    /**
     * @public x
     * @brief Get the robot's x-coordinate
     * @returns X-coordinate of the robot's position (inches)
     */
    double x();

    /**
     * @public y
     * @brief Get the robot's y-coordinate
     * @returns Y-coordinate of the robot's position (inches)
     */
    double y();

    /**
     * @public rotation
     * @brief Get the robot's rotation
     * @returns Rotation of the robot (radians)
     */
    double rotation();

    friend int wire_control(void* o); // Tested on Friday, April 11th, 2025
    friend int arcade_control(void* o); // Tested on Friday, April 11th, 2025
    friend int curvature_control(void* o); // Tested on Friday, April 11th, 2025
    friend int tank_control(void* o); // Tested on Friday, April 11th, 2025
};

/**
 * @public wire_control
 * @brief Control the robot using Drive-By-Wire
 * @param o Pointer to the Chassis object
 * @relatesalso Chassis
 * @details
 * The wire_control method allows the robot to be controlled using a drive by wire program.
 * This is useful for in-game control of the robot when chassis imbalance occurs, or when for any reason the robot
 * needs to lock its heading.
 * 
 * Mechanical issues:
 * - Friction: if screw joints or kepts nuts are not properly tuned or they loosen during the game, there may be some drift bias.
 * This is usually minimal in the friction case so DBW is usually not needed.
 * - Overheating: if one side of the robot heats more than another (e.g., due to driver preference of drifting around the field
 * or physical obstruction for heat dissipation), the robot may drift to one side. This is an intermediate bias.
 * - Motor disconnection or failure: if one side of the robot has a motor disconnection or failure, the robot may drift to one side.
 * This is a major bias which will almost always warrant the use of DBW.
 * 
 * Heading lock:
 * - If the robot is guarding any kind of goal, DBW will make it difficult for the opponent to push the robot except sideways as the
 * wire system will actively resist any movement it can (which is all movement except orthogonal to the robot's heading).
 * 
 * @note
 * Drive-By-Wire is a method of controlling the robot using a controller that imposes a controller in between the user and the actuator.
 * This allows for more precise control of the robot's movement and can help to mitigate the effects of bias.
 * 
 * DBW is inspired by modern aircraft "Fly-By-Wire" systems, which use a computer to control the aircraft's flight surfaces.
 * 
 * Internally, DBW maintains a target heading and uses a PID controller to adjust the robot's movement based on the user's input.
 * The internal PID has parameters set so that it is always active and behaves like a traditional controller (no Clegg integration,
 * integral limits, etc.) with a small change: since PID is a feedback controller, there is inevitably some lag in the response,
 * so the robot will feel sluggish when turning. Thus, the PID also implements a small feedforward, "anticipative" term that
 * is added to the turning output based on the user's input which allows for more responsive control.
 * 
 * Drive-by-wire feels similar in response to a curvature drive (and not arcade drive).
 */
int wire_control(void* o);

/**
 * @public arcade_control
 * @brief Control the robot using arcade control
 * @param o Pointer to the Chassis object
 * @relatesalso Chassis
 * @details
 * The arcade_control method allows the robot to be controlled using arcade control.
 * 
 * Left side joystick controls forward/backward movement, while the right side joystick controls turning.
 */
int arcade_control(void* o);

/**
 * @public curvature_control
 * @brief Control the robot using curvature control
 * @param o Pointer to the Chassis object
 * @relatesalso Chassis
 * @details
 * The curvature_control method allows the robot to be controlled using curvature control.
 * 
 * Left side joystick controls forward/backward movement, while the right side joystick controls turn curvature.
 * This is similar to arcade control, but the turning response feels different.
 */
int curvature_control(void* o);

/**
 * @public tank_control
 * @brief Control the robot using tank control
 * @param o Pointer to the Chassis object
 * @relatesalso Chassis
 * @details
 * The tank_control method allows the robot to be controlled using tank control.
 * 
 * Left side joystick controls left motor group, while the right side joystick controls right motor group.
 * 
 * This is the most difficult control scheme to use, as it requires the user to control both sides of the robot independently;
 * however, when mastered, it is the most precise control scheme which grants the driver complete control over drifting and turning.
 */
int tank_control(void* o);

#endif // #ifndef CHASSIS_HPP

/* ---- SKETCH FOR CODE INTEGRATION  ---- */

/*
#include devices
#include glob_variables
#include pid
int highstakes_control(void* o) {
    
}

int highstakes_lb() {

}
*/