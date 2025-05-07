#include <pursuit.hpp>
#include <path.hpp>
#include <algorithm>
#include <utility>
#include <vector>
#include <cmath>
#include <vex.h>

double Pursuit::circle_line_intersection(double x1, double y1, double x2, double y2, double x_bot, double y_bot, double r) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double fx = x1 - x_bot;
    double fy = y1 - y_bot;
    double a = dx * dx + dy * dy;
    double b = (fx * dx + fy * dy) * 2.0;
    double c = fx * fx + fy * fy - r * r;
    double determinant = b * b - 4.0 * a * c;
    if (determinant < 0) {
        return -1.0;
    }
    double sqrt_determinant = sqrt(determinant);
    double t1 = (-b + sqrt_determinant) / (a * 2.0);
    double t2 = (-b - sqrt_determinant) / (a * 2.0);
    if (t1 >= 0 and t1 <= 1) {
        return t1;
    }
    if (t2 >= 0 and t2 <= 1) {
        return t2;
    }
    return -1.0;
}

Pursuit::Pursuit(Path path, double lookahead_distance): lookahead(lookahead_distance), path(path), last_lookahead(0, 0) {
    last_found_idx = 0;
}

Pursuit::Pursuit(std::vector<double> x, std::vector<double> y, int point_count, double lookahead_distance): lookahead(lookahead_distance), path(x, y, point_count), last_lookahead(0, 0) {
    last_found_idx = 0;
}

const double & Pursuit::lookahead_distance() const {
    return lookahead;
}

double & Pursuit::lookahead_distance(double new_lookahead) {
    return lookahead = new_lookahead;
}

Coordinate2D Pursuit::get_target(double from_x, double from_y) {
    std::vector<std::pair<int, Coordinate2D>> found_targets;
    if (last_found_idx == path.points.size() - 1) {
        return path.points.back();
    }
    for (int segment = last_found_idx; segment < path.points.size() - 1; ++segment) {
        auto [x1, y1] = path.points[segment];
        auto [x2, y2] = path.points[segment + 1];
        double t = circle_line_intersection(x1, y1, x2, y2, from_x, from_y, lookahead);
        if (t >= 0 and t <= 1) {
            double x = x1 * (1 - t) + x2 * t;
            double y = y1 * (1 - t) + y2 * t;
            last_found_idx = segment;
            return last_lookahead = Coordinate2D(x, y);
        }
    }
    return last_lookahead;
}

std::pair<std::pair<double, double>, double> Pursuit::get_relative_steering(double from_x, double from_y, double theta_bot, double width_bot, double norm) {
    Coordinate2D target = get_target(from_x, from_y);
    double w = width_bot; // Robot width
    double t0 = theta_bot; // Initial heading
    double dx = target.x - from_x; // Relative x
    double dy = target.y - from_y; // Relative y
    double alpha = atan2(dy, dx) - t0; // Relative angle from the robot to the target
    double d = sqrt(dx * dx + dy * dy); // Distance to the target (should be lookahead_distance most of the time)
    double sine = sin(alpha); // Sine of the relative angle
    if (fabs(sine) < 1e-7) { // Straight line- infinite radius
        return (cos(alpha) > 0 ? std::make_pair(std::make_pair(norm * 0.5, norm * 0.5), 1e9) : std::make_pair(std::make_pair(-norm * 0.5, -norm * 0.5), 1e9)); // Straight line
    }
    double r_c = d / (sine * 2); // Central radius
    double left_steering = fabs(r_c - w * 0.5); // Radius 1
    double right_steering = fabs(r_c + w * 0.5); // Radius 2

    // double max_steer = left_steering < right_steering ? right_steering : left_steering; 
    // double scale = norm / max_steer; // normed so max(|l|, |r|) = norm
    double scale = norm / (fabs(left_steering) + fabs(right_steering)); // normed so |l| + |r| = norm
    if (cos(alpha) < 0) {
        scale = -scale;
    }
    printf("(%.5f, %.5f)\n", target.x, target.y);
    // printf("Pursuit: (%.3f, %.3f) -> (%.3f, %.3f) with alpha=%.3f, steering=(%.3f, %.3f)\n", from_x, from_y, target.x, target.y, alpha, left_steering * scale, right_steering * scale);
    return std::make_pair(std::make_pair(left_steering * scale, right_steering * scale), fabs(r_c));
}

bool Pursuit::terminated(double x_bot, double y_bot) {
    bool progress_condition = ((double) (last_found_idx + 1) / (double) path.points.size()) >= progress_threshold;
    auto [x, y] = path.points.back();
    const double dx = x - x_bot;
    const double dy = y - y_bot;
    const double distance = sqrt(dx * dx + dy * dy);
    bool distance_condition = distance < distance_threshold;
    return progress_condition and distance_condition;
}

double Pursuit::progress() {
    return (double) last_found_idx / (double) path.points.size();
}

// Simulator: https://www.desmos.com/calculator/6uq283vk0l
// The simulator used the path: {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {-10, 0}, {0, 0}}

// Last reviewed on Saturday, February 22nd, 2025