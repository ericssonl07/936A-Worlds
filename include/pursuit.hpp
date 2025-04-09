#pragma once
#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <path.hpp>
#include <vector>
#include <utility>

class Pursuit {
    friend int autonomous();
    friend class Chassis;
    int last_found_idx;
    double lookahead;
    static const double progress_threshold = 0.95;
    static const double distance_threshold = 1.0;
    static double circle_line_intersection(double x1, double y1, double x2, double y2, double x_bot, double y_bot, double r);
    Path path;
    Coordinate2D last_lookahead;
public:
    Pursuit(Path path, double lookahead_distance);
    Pursuit(std::vector<double> x, std::vector<double> y, int num_points, double lookahead_distance);
    const double & lookahead_distance() const;
    double & lookahead_distance(double new_lookahead);
    Coordinate2D get_target(double x_bot, double y_bot);
    std::pair<std::pair<double, double>, double> get_relative_steering(double x_bot, double y_bot, double theta_bot, double width_bot, double norm = 1.0);
    bool terminated(double x_bot, double y_bot);
    double progress();
};

#endif // #ifndef PURE_PURSUIT_HPP