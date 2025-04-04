#include <path.hpp>
#include <limits>
#include <vector>
#include <cmath>


// Coordinate 2D
Coordinate2D::Coordinate2D(): x(0), y(0) {}
Coordinate2D::Coordinate2D(double x, double y): x(x), y(y) {}
Coordinate2D::Coordinate2D(std::pair<double, double> point): x(point.first), y(point.second) {}
Coordinate2D Coordinate2D::operator+ (const Coordinate2D& other) { return Coordinate2D(x + other.x, y + other.y); }
Coordinate2D Coordinate2D::operator- (const Coordinate2D& other) { return Coordinate2D(x - other.x, y - other.y); }
Coordinate2D Coordinate2D::operator* (double scalar) { return Coordinate2D(x * scalar, y * scalar); }
Coordinate2D Coordinate2D::operator/ (double scalar) { return Coordinate2D(x / scalar, y / scalar); }
Coordinate2D operator* (double scalar, const Coordinate2D& point) { return Coordinate2D(scalar * point.x, scalar * point.y); }
std::ostream& operator<< (std::ostream& os, const Coordinate2D& point) { os << '(' << point.x << ", " << point.y << ')'; return os; }


// Cubic Expression
CubicExpression::CubicExpression(double a, double b, double c, double d): a(a), b(b), c(c), d(d) {}
std::ostream& operator<< (std::ostream& os, const CubicExpression& c) { os << c.a << "x^3 + " << c.b << "x^2 + " << c.c << "x + " << c.d; return os; }


// CubicSpline
std::vector<double> CubicSpline::linspace(double start, double end, int point_count) {
    std::vector<double> result;
    if (point_count <= 0) {
        throw std::logic_error("Number of points must be positive");
    }
    double step = (end - start) / (point_count - 1);
    for (int i = 0; i < point_count; ++i) {
        result.push_back(start + i * step);
    }
    return result;
}

int CubicSpline::get_segment(double of) {
    int idx = coefficients.size() - 1;
    while (piecewise_boundaries[idx] > of) {
        --idx;
        if (idx == 0) {
            return 0;
        }
    }
    return idx;
}

CubicSpline::CubicSpline(std::vector<double> x_values, std::vector<double> y_values): piecewise_boundaries(x_values) {
    if (x_values.size() != y_values.size()) {
        std::cerr << "Inhomogeneous vector shapes\n";
        throw std::logic_error("Inhomogeneous vector shapes");
    }
    piecewise_boundaries = x_values;
    int pieces_count = x_values.size() - 1;
    coefficients = std::vector<CubicExpression>(pieces_count);
    operations = zeros(pieces_count * 4, pieces_count * 4 + 1);
    operations[0][0] = x_values[0] * 6.0;
    operations[0][1] = 2.0;
    operations[pieces_count * 4 - 3][pieces_count * 4 - 4] = pow(x_values[pieces_count - 1], 3.0);
    operations[pieces_count * 4 - 3][pieces_count * 4 - 3] = pow(x_values[pieces_count - 1], 2.0);
    operations[pieces_count * 4 - 3][pieces_count * 4 - 2] = x_values[pieces_count - 1];
    operations[pieces_count * 4 - 3][pieces_count * 4 - 1] = 1.0;
    operations[pieces_count * 4 - 2][pieces_count * 4 - 4] = pow(x_values[pieces_count], 3.0);
    operations[pieces_count * 4 - 2][pieces_count * 4 - 3] = pow(x_values[pieces_count], 2.0);
    operations[pieces_count * 4 - 2][pieces_count * 4 - 2] = x_values[pieces_count];
    operations[pieces_count * 4 - 2][pieces_count * 4 - 1] = 1.0;
    operations[pieces_count * 4 - 1][pieces_count * 4 - 4] = x_values[pieces_count] * 6.0;
    operations[pieces_count * 4 - 1][pieces_count * 4 - 3] = 2.0;
    for (int bounding_box_number = 0; bounding_box_number < pieces_count - 1; ++bounding_box_number) {
        int relative_row_idx = bounding_box_number * 4 + 1, relative_column_idx = bounding_box_number * 4;
        operations[relative_row_idx][relative_column_idx] = pow(x_values[bounding_box_number], 3.0);
        operations[relative_row_idx][relative_column_idx + 1] = pow(x_values[bounding_box_number], 2.0);
        operations[relative_row_idx][relative_column_idx + 2] = x_values[bounding_box_number];
        operations[relative_row_idx][relative_column_idx + 3] = 1.0;
        operations[relative_row_idx + 1][relative_column_idx] = pow(x_values[bounding_box_number + 1], 3.0);
        operations[relative_row_idx + 1][relative_column_idx + 1] = pow(x_values[bounding_box_number + 1], 2.0);
        operations[relative_row_idx + 1][relative_column_idx + 2] = x_values[bounding_box_number + 1];
        operations[relative_row_idx + 1][relative_column_idx + 3] = 1.0;
        operations[relative_row_idx + 2][relative_column_idx] = pow(x_values[bounding_box_number + 1], 2.0) * 3.0;
        operations[relative_row_idx + 2][relative_column_idx + 1] = x_values[bounding_box_number + 1] * 2.0;
        operations[relative_row_idx + 2][relative_column_idx + 2] = 1.0;
        operations[relative_row_idx + 2][relative_column_idx + 4] = -pow(x_values[bounding_box_number + 1], 2.0) * 3.0;
        operations[relative_row_idx + 2][relative_column_idx + 5] = -x_values[bounding_box_number + 1] * 2.0;
        operations[relative_row_idx + 2][relative_column_idx + 6] = -1.0;
        operations[relative_row_idx + 3][relative_column_idx] = x_values[bounding_box_number + 1] * 6.0;
        operations[relative_row_idx + 3][relative_column_idx + 1] = 2.0;
        operations[relative_row_idx + 3][relative_column_idx + 4] = -x_values[bounding_box_number + 1] * 6.0;
        operations[relative_row_idx + 3][relative_column_idx + 5] = -2.0;
    }
    for (int idx = 0; idx < pieces_count; ++idx) {
        int i = idx * 4 + 1, j = pieces_count * 4;
        operations[i][j] = y_values[idx];
        operations[i + 1][j] = y_values[idx + 1];
    }
    Matrix reduced_form = operations.rref();
    std::vector<double> values(pieces_count * 4, 0.0);
    for (int i = 0; i < pieces_count * 4; ++i) {
        values[i] = reduced_form[i][pieces_count * 4];
    }
    for (int i = 0; i < pieces_count; ++i) {
        coefficients[i] = CubicExpression(values[i * 4], values[i * 4 + 1], values[i * 4 + 2], values[i * 4 + 3]);
    }
}

std::vector<double> CubicSpline::operator() (std::vector<double> values) {
    std::vector<double> result;
    for (auto value: values) {
        CubicExpression coefficient = coefficients[get_segment(value)];
        double a = coefficient.a, b = coefficient.b, c = coefficient.c, d = coefficient.d;
        double cubic_value = a * value * value * value + b * value * value + c * value + d;
        result.push_back(cubic_value);
    }
    return result;
}


// Path
void Path::construct_path(std::vector<double> x, std::vector<double> y, int point_count) {
    if (point_count == -1) {
        point_count = x.size() * 25;
    }
    if (x.size() != y.size()) {
        std::cerr << "Inhomogeneous vector lengths\n";
        throw std::logic_error("Inhomogeneous vector lengths");
    }
    std::vector<double> parameter(x.size(), 0.0);
    for (int i = 1; i < parameter.size(); ++i) {
        parameter[i] = (double) i;
    }
    CubicSpline x_spline(parameter, x);
    CubicSpline y_spline(parameter, y);
    std::vector<double> p_interp = CubicSpline::linspace(0, parameter.size() - 1, point_count);
    std::vector<double> x_interp = x_spline(p_interp);
    std::vector<double> y_interp = y_spline(p_interp);
    for (int i = 0; i < point_count; ++i) {
        points.push_back(Coordinate2D(x_interp[i], y_interp[i]));
    }
}

Coordinate2D Path::operator [] (int idx) {
    return points[idx];
}

std::size_t Path::size() {
    return points.size();
}

Path::Path(): points() {}

Path::Path(std::vector<double> x, std::vector<double> y, int point_count): points() {
    construct_path(x, y, point_count);
}

Path::Path(std::vector<std::pair<double, double>> points, int point_count): points() {
    std::vector<double> x_values;
    std::vector<double> y_values;
    for (auto [x, y]: points) {
        x_values.push_back(x);
        y_values.push_back(y);
    }
    construct_path(x_values, y_values, point_count);
}

void verify_filename(std::string filename) {
    int idx = filename.find(".path");
    if (idx == std::string::npos) {
        std::cerr << "Invalid file extension\n";
        throw std::logic_error("Invalid file extension");
    }
    if (filename.substr(idx, 5) != ".path") {
        std::cerr << "Invalid file extension\n";
        throw std::logic_error("Invalid file extension");
    }
}

Path::Path(std::string filename): points() {
    verify_filename(filename);
    std::fstream file;
    file.open(filename, std::ios::in | std::ios::binary);
    std::vector<Coordinate2D> coordinates;
    char flag;
    file >> flag;
    while (flag == 'P') {
        double x_val, y_val;
        file >> x_val >> y_val >> flag;
        Coordinate2D point(x_val, y_val);
        coordinates.push_back(point);
    }
    points = std::move(coordinates);
}

void Path::serialize(std::string filename) {
    std::fstream file;
    file.open(filename, std::ios::out | std::ios::binary);
    for (Coordinate2D point : points) {
        file << 'P' << " " << point.x << " " << point.y << std::endl;
    }
    file << 'E' << std::endl;
}