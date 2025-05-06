#pragma once
#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <path.hpp>
#include <vector>
#include <utility>

/**
 * @class Pursuit
 * @file pursuit.hpp
 * @author @ericssonl07
 * @date 2025-04-20
 * @brief
 * This class implements a pure pursuit algorithm for path following in robotics.
 * @details
 * The Pursuit class is designed to follow a given path using the pure pursuit algorithm.
 * It calculates the target point on the path based on the robot's current position and orientation.
 * Then, based on the relative position of the target point and the robot's orientation and geometry,
 * it calculates the steering angle required to follow the path in the form of a pair of left/right voltages.
 * Additionally, Pursuit provides the radius of the osculating circle to the target point, which can be used
 * for dynamic velocity control based on centripetal acceleration.
 * @example
 * ```cpp
 * // Example usage of the Pursuit class:
 * double lookahead = 12.0; // Lookahead distance in inches
 * Pursuit pursuit(path, lookahead); // Create a Pursuit object with a given path and lookahead distance
 * double x_bot = 5.0; // Current x position of the robot
 * double y_bot = 3.0; // Current y position of the robot
 * double theta_bot = M_PI / 4; // Current orientation of the robot in radians
 * double width_bot = 2.0; // Width of the robot
 * double norm = 12.0; // Normalization factor for voltage
 * auto [steering, curvature] = pursuit.get_relative_steering(x_bot, y_bot, theta_bot, width_bot, norm); // Get the relative steering and curvature
 * auto [left_voltage, right_voltage] = steering; // Extract left and right voltages
 * ```
 * @see
 * Path
 * 
 * CubicSpline
 * 
 * Chassis
 */
class Pursuit {
/**
 * @privatesection
 */

    friend int blue_ringrush();
    
    friend class Chassis;
    
    /**
     * @private last_found_idx
     * @brief Index of the last found point on the path
     * @details This index is used to keep track of the last point on the path that was found during the pursuit.
     * It is used to optimize the search for the next target point on the path by skipping points that are behind
     * the robot in the path.
     */
    int last_found_idx;

    /**
     * @private lookahead
     * @brief Lookahead distance for the pursuit algorithm (inches)
     * @details This distance is used to determine how far ahead on the path the robot should look for the target point.
     * @note
     * A higher lookahead distance will result in smoother turns, but may cause the robot to miss the path if it is too far ahead.
     * A lower lookahead distance will result in sharper turns, but may cause the robot to oscillate or overshoot the path.
     */
    double lookahead;

    /**
     * @private progress_threshold
     * @brief Progress threshold for the pursuit algorithm
     * @details This threshold is used to determine when the robot has made sufficient progress along the path (part of the termination condition).
     */
    static const double progress_threshold = 0.95;

    /**
     * @private distance_threshold
     * @brief Distance threshold for the pursuit algorithm
     * @details This threshold is used to determine when the robot has reached the target point on the path (part of the termination condition).
     */
    static const double distance_threshold = 1.0;

    /**
     * @private circle_line_intersection
     * @brief Calculate the intersection of a line and a circle
     * @param x1 The x-coordinate of the first point on the line
     * @param y1 The y-coordinate of the first point on the line
     * @param x2 The x-coordinate of the second point on the line
     * @param y2 The y-coordinate of the second point on the line
     * @param x_bot The x-coordinate of the center of the circle
     * @param y_bot The y-coordinate of the center of the circle
     * @param r The radius of the circle
     * @details This method calculates the intersection points between a line defined by two points and a circle defined by its center and radius.
     * 
     * It returns the parameter t in [0.0, 1.0] that corresponds to the progress along the line where the intersection occurs.
     * 
     * We parametrize the line by (1-t)P1 + tP2, 0<=t<=1, where P1 and P2 are the two points defining the line.
     * @note Fundamentally, the equation reduces to a quadratic equation in t. The result can be classified depending on the discriminant:
     * - If the discriminant is negative, there is no intersection. We return -1.
     * - If the discriminant is zero, there is one intersection point. We return t.
     * - If the discriminant is positive, there are two intersection points. We return the larger t, which is the one that is closer to the
     * second point P2.
     * 
     * The function is static because it does not depend on the state of the Pursuit object.
     * @returns The parameter t in [0.0, 1.0] that corresponds to the progress along the line where the intersection occurs, or -1 if there is no intersection.
     */
    static double circle_line_intersection(double x1, double y1, double x2, double y2, double x_bot, double y_bot, double r);

    /**
     * @private path
     * @brief Path object representing the path to be followed
     * @details This object contains the points that define the path to be followed by the robot.
     */
    Path path;

    /**
     * @private last_lookahead
     * @brief Last lookahead point on the path
     * @details This point is used to keep track of the last lookahead point on the path that was found during the pursuit.
     * @note The last_lookahead point is used as a backup in case the current lookahead point is not found.
     */
    Coordinate2D last_lookahead;

/**
 * @publicsection
 */
public:

    /**
     * @public constructor
     * @brief Default constructor for Pursuit
     * @details Initializes the Pursuit object with a default lookahead distance of 12.0 inches.
     * @param path The path to be followed by the robot
     * @param lookahead_distance The lookahead distance for the pursuit algorithm (inches).
     * The lookahead distance is used to determine how far ahead on the path the robot should look for the target point.
     * A higher lookahead distance will result in smoother turns, but may cause the robot to miss the path if it is too far ahead.
     * A lower lookahead distance will result in sharper turns, but may cause the robot to oscillate or overshoot the path.
     */
    Pursuit(Path path, double lookahead_distance);

    /**
     * @public constructor
     * @brief Constructor for Pursuit
     * @param x The x-coordinates of the points used to construct the path
     * @param y The y-coordinates of the points used to construct the path
     * @param num_points The number of points to generate. If -1, the number of points is determined by size(points) * 10.
     * @param lookahead_distance The lookahead distance for the pursuit algorithm (inches).
     * The lookahead distance is used to determine how far ahead on the path the robot should look for the target point.
     * A higher lookahead distance will result in smoother turns, but may cause the robot to miss the path if it is too far ahead.
     * A lower lookahead distance will result in sharper turns, but may cause the robot to oscillate or overshoot the path.
     * @details This constructor initializes the Pursuit object with the specified x and y values.
     * It constructs the path by generating a set of points along the path using cubic spline interpolation.
     * @throws std::logic_error if the x and y values are not of the same size
     */
    Pursuit(std::vector<double> x, std::vector<double> y, int num_points, double lookahead_distance);

    /**
     * @public lookahead_distance
     * @brief Get the lookahead distance for the pursuit algorithm
     * @details This method returns the lookahead distance for the pursuit algorithm as a const reference.
     * @returns The lookahead distance for the pursuit algorithm (inches) as a const reference.
     */
    const double & lookahead_distance() const;

    /**
     * @public lookahead_distance
     * @brief Set the lookahead distance for the pursuit algorithm
     * @param new_lookahead The new lookahead distance for the pursuit algorithm (inches).
     * The lookahead distance is used to determine how far ahead on the path the robot should look for the target point.
     * A higher lookahead distance will result in smoother turns, but may cause the robot to miss the path if it is too far ahead.
     * A lower lookahead distance will result in sharper turns, but may cause the robot to oscillate or overshoot the path.
     * @details This method sets the lookahead distance for the pursuit algorithm.
     * @returns A reference to the new lookahead distance for the pursuit algorithm (inches) as a non-const reference.
     */
    double & lookahead_distance(double new_lookahead);

    /**
     * @public get_target
     * @brief Get the target point on the path from the robot's current position
     * @param x_bot The x-coordinate of the robot's current position (inches)
     * @param y_bot The y-coordinate of the robot's current position (inches)
     * @details This method calculates the target point on the path based on the robot's current position and orientation.
     * It iterates forward through the path's waypoints from the last found index to find the first point that is within the lookahead distance.
     * At each step, it calls circle_line_intersection to check if the line from the robot's position to the current point intersects with
     * the circle defined by the lookahead distance.
     * If an intersection is found, it returns the target point on the path.
     * If no intersections are found by the end, it returns the last found intersection as a backup.
     * @returns The target point on the path as a Coordinate2D object.
     */
    Coordinate2D get_target(double x_bot, double y_bot);

    /**
     * @public get_relative_steering
     * @brief Get the relative steering angle and curvature for the robot to follow the path
     * @param x_bot The x-coordinate of the robot's current position (inches)
     * @param y_bot The y-coordinate of the robot's current position (inches)
     * @param theta_bot The orientation of the robot (radians)
     * @param width_bot The width of the robot (inches)
     * @param norm The normalization factor (default is 1.0)- so that |L|+|R|=norm.
     * We consider modifying this so that max(|L|, |R|)=norm- to be decided.
     * @todo Consider modifying the normalization factor to max(|L|, |R|)=norm for better control.
     * @details This method calculates the steering angle and curvature required for the robot to follow the path.
     * It uses the target point obtained from get_target and calculates the relative steering angle based on the robot's
     * current position, orientation, and geometry.
     * It returns a pair containing the left and right voltages as well as the curvature of the path at the target point.
     * 
     * Return format: [[left, right], central radius]
     * @returns A pair containing a pair of left and right voltages, and a double representing the curvature.
     * Format: [[left, right], central radius]
     */
    std::pair<std::pair<double, double>, double> get_relative_steering(double x_bot, double y_bot, double theta_bot, double width_bot, double norm = 1.0);

    /**
     * @public terminated
     * @brief Check if the robot has terminated the pursuit
     * @param x_bot The x-coordinate of the robot's current position (inches)
     * @param y_bot The y-coordinate of the robot's current position (inches)
     * @details This method checks if the robot has terminated the pursuit based on the progress and distance thresholds.
     * The termination condition is: found_idx / path.size() > progress_threshold and distance(x_bot, y_bot) < distance_threshold.
     * @note The termination condition is: found_idx / path.size() > progress_threshold and distance(x_bot, y_bot) < distance_threshold.
     * @returns True if the pursuit is terminated, false otherwise.
     */
    bool terminated(double x_bot, double y_bot);

    /**
     * @public progress
     * @brief Get the progress of the pursuit
     * @returns last_found_idx / path.size() as a double- the progress of the pursuit ranging from 0.0 (not started) to 1.0 (completed).
     */
    double progress();
};

#endif // #ifndef PURE_PURSUIT_HPP