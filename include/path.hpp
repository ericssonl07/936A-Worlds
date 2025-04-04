#pragma once
#ifndef PATH_HPP
#define PATH_HPP

#include <matrix.hpp>
#include <vector>
#include <fstream>

struct Coordinate2D {
    double x, y;
    Coordinate2D();
    Coordinate2D(double x, double y);
    Coordinate2D(std::pair<double, double> point);
    Coordinate2D(const Coordinate2D& other) = default;
    Coordinate2D(Coordinate2D&& other) = default;
    Coordinate2D& operator= (const Coordinate2D& other) = default;
    Coordinate2D& operator= (Coordinate2D&& other) = default;
    ~Coordinate2D() = default;
    Coordinate2D operator+ (const Coordinate2D& other);
    Coordinate2D operator- (const Coordinate2D& other);
    Coordinate2D operator* (double scalar);
    Coordinate2D operator/ (double scalar);
    friend Coordinate2D operator* (double scalar, const Coordinate2D& point);
    friend std::ostream& operator<< (std::ostream& os, const Coordinate2D& point);
};

struct CubicExpression {
    double a, b, c, d;
    CubicExpression() = default;
    CubicExpression(double a, double b, double c, double d);
    CubicExpression(const CubicExpression& other) = default;
    CubicExpression(CubicExpression&& other) = default;
    CubicExpression& operator= (const CubicExpression& other) = default;
    CubicExpression& operator= (CubicExpression&& other) = default;
    ~CubicExpression() = default;
    friend std::ostream& operator<< (std::ostream& os, const CubicExpression& expression);
};

class CubicSpline {
    friend class Path;
    Matrix operations;
    std::vector<CubicExpression> coefficients;
    std::vector<double> piecewise_boundaries;
    static std::vector<double> linspace(double start, double end, int point_count);
public:
    int get_segment(double of);
    CubicSpline(std::vector<double> x_values, std::vector<double> y_values);
    std::vector<double> operator() (std::vector<double> values);
};

class Path {
    friend class Pursuit;
    friend class Chassis;
    std::vector<Coordinate2D> points;
    void construct_path(std::vector<double> x, std::vector<double> y, int point_count);
public:
    Coordinate2D operator [] (int idx);
    std::size_t size();
    Path();
    Path(std::vector<double> x, std::vector<double> y, int point_count = -1);
    Path(std::vector<std::pair<double, double>> points, int point_count = -1);
    Path(std::string filename);
    Path(const Path& other) = default;
    Path(Path&& other) = default;
    Path& operator= (const Path& other) = default;
    Path& operator= (Path&& other) = default;
    void serialize(std::string filename);
};

#endif // #ifndef PATH_HPP