#ifndef VEXLIBRARY_VIRTUAL_ODOMETRY_HPP
#define VEXLIBRARY_VIRTUAL_ODOMETRY_HPP

#include <vex.h>
#include <motor_group.hpp>
#include <vector>
#include <cmath>

/**
 * @class Odometry
 * @file odometry.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This class represents an odometry system for a robot, providing methods for tracking its position and orientation.
 * @details
 * The Odometry class is designed to work with a set of 2 tracking wheels and an inertial sensor.
 * Work is in progress to add support for all tracking configurations.
 * 
 * The class manages its own thread for tracking and displaying the position. Display is currently comment out.
 * The tracking thread updates every 50ms.
 * @note
 * The class uses the readings from the tracking wheels and the inertial sensor to calculate the robot's position and orientation.
 * It utilizes the assumption of "local circularity" (think "local linearity" in calculus) to simplify the geometry into circular arcs.
 * Then, by choosing a convenient coordinate system offset from global by robot heading + change in heading * 0.5, we can find the actual
 * change globally.
 * @attention
 * The class expects an IMU sensor that is calibrated and properly aligned with the robot's frame of reference
 * (all offsets accounted for in the vex::inertial constructor).
 * 
 * Additionally, the IMU should follow mathematical convention of positive rotations being counter-clockwise.
 * Set this using the vex::turnType::left specifier in the vex::inertial constructor (see example usage).
 * 
 * This class is not intended to be used directly- `Odometry` itself is managed by the `Chassis` class.
 * Though in theory, it could be used independently, it is not recommended to preserve the modularity and cleanness of the code.
 * @example
 * ```cpp
 * // Example usage of the Odometry class (independent usage not recommended- recommended usage through Chassis):
 * vex::rotation forward_track(vex::PORT1, vex::gearSetting::ratio18_1, false);
 * vex::rotation side_track(vex::PORT2, vex::gearSetting::ratio18_1, false);
 * vex::inertial imu(vex::PORT3, vex::turnType::left);
 * imu.calibrate(); // Calibrate the IMU
 * while (imu.is_calibrating()) { // Wait for calibration to finish
 *   vex::this_thread::sleep_for(100); // During this period, ensure the robot is stationary
 * }
 * Odometry odometry(&forward_track, &side_track, &imu, 0.0, 0.0, 2.5);
 * odometry.set_pose(0.0, 0.0, 0.0); // Set initial position
 * odometry.x(); // Get the current x position
 * odometry.y(); // Get the current y position
 * odometry.rotation(); // Get the current rotation
 * ```
 * @see
 * vex::rotation
 * 
 * vex::inertial
 * 
 * The [Sigbots Odometry page](https://wiki.purduesigbots.com/software/odometry)
 */
class Odometry {

/**
 * @privatesection
 */
private:

    /**
     * @private track
     * @brief Track the robot's position and orientation
     * @note This function is run in a separate thread to continuously update the robot's position and orientation.
     * @returns 0- never returns
     */
    friend int track(void* o);

    /**
     * @private display
     * @brief Display the robot's position and orientation
     * @note This function is run in a separate thread to continuously update the display with the robot's position and orientation.
     * @returns 0- never returns
     */
    friend int display(void* o);

    /**
     * @private tracking_thread
     * @brief Thread for tracking the robot's position and orientation
     * @note This thread runs the track function to continuously update the robot's position and orientation.
     * @see vex::thread
     */
    vex::thread tracking_thread;

    /**
     * @private displaying_thread
     * @brief Thread for displaying the robot's position and orientation
     * @note This thread runs the display function to continuously update the display with the robot's position and orientation.
     * @see vex::thread
     */
    vex::thread displaying_thread;

    /**
     * @private forward_track
     * @brief Forward tracking wheel
     * @note This wheel is parallel to the robot's velocity vector.
     * It is used to track the robot's forward/backward movement.
     * @see vex::rotation
     */
    vex::rotation* forward_track;

    /**
     * @private side_track
     * @brief Side tracking wheel
     * @note This wheel is perpendicular to the robot's velocity vector.
     * It is used to track the robot's left/right movement.
     * @see vex::rotation
     */
    vex::rotation* side_track;

    /**
     * @private imu
     * @brief Inertial sensor
     * @note This sensor is used to track the robot's rotation.
     * It is expected to be calibrated and properly aligned with the robot's frame of reference.
     * Also, it should follow mathematical convention of positive rotations being counter-clockwise.
     * @see
     * vex::inertial
     * 
     * vex::inertial::calibrate
     */
    vex::inertial* imu;

    /**
     * @private left_right_offset
     * @brief Offset of the left/right tracking wheel to the tracking center
     * @note Left is positive, right is negative. Given in inches.
     */
    double left_right_offset; // left is positive

    /**
     * @private forward_back_offset
     * @brief Offset of the front/back tracking wheel to the tracking center
     * @note Forward is negative, backward is positive. Given in inches.
     */
    double forward_back_offset; // back is positive

    /**
     * @private tracking_radius
     * @brief Radius of the tracking wheels
     * @note This is the distance from the center of the tracking wheel to its outer edge, in inches.
     */
    double tracking_radius;

    /**
     * @private x_position
     * @brief X-coordinate of the robot's position
     * @note This is the distance from the center of the robot to the tracking center, in inches.
     */
    double x_position;

    /**
     * @private y_position
     * @brief Y-coordinate of the robot's position
     * @note This is the distance from the center of the robot to the tracking center, in inches.
     */
    double y_position;

    /**
     * @private rotation_value
     * @brief Rotation of the robot
     * @note This is the angle of the robot's rotation, in radians.
     */
    double rotation_value;

    /**
     * @private get_forward
     * @brief Get the forward tracking wheel's rotation
     * @note This function returns the rotation of the forward tracking wheel, in inches.
     * @returns The rotation of the forward tracking wheel, in inches.
     */
    double get_forward();

    /**
     * @private get_side
     * @brief Get the side tracking wheel's rotation
     * @note This function returns the rotation of the side tracking wheel, in inches.
     * @returns The rotation of the side tracking wheel, in inches.
     */
    double get_back();

/**
 * @publicsection
 */
public:

    /**
     * @public constructor
     * @brief Constructor for the Odometry class
     * @param forward_track Forward tracking wheel (parallel to the robot's velocity vector)
     * @param side_track Side tracking wheel (perpendicular to the robot's velocity vector)
     * @param imu Inertial sensor (calibrated and with vex::turnType::left specifier)
     * @param left_right_offset Offset of the left/right tracking wheel to the tracking center (left is positive, right is negative)
     * @param forward_back_offset Offset of the front/back tracking wheel to the tracking center (forward is negative, backward is positive)
     * @param tracking_radius Radius of the tracking wheels (distance from the center of the tracking wheel to its outer edge)
     */
    Odometry(vex::rotation* forward_track, vex::rotation* side_track, vex::inertial* imu,
             double left_right_offset,  double forward_back_offset, double tracking_radius);

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

    /**
     * @public reset
     * @brief Reset the robot's position and rotation
     * @note This function resets the robot's position and rotation to (0, 0, 0).
     */
    void reset();

    /**
     * @public set_pose
     * @brief Set the robot's position and rotation
     * @param x X-coordinate of the robot's position (inches)
     * @param y Y-coordinate of the robot's position (inches)
     * @param rotation Rotation of the robot (radians)
     */
    void set_pose(double x, double y, double rotation);
};

#endif // #ifndef VEXLIBRARY_VIRTUAL_ODOMETRY_HPP