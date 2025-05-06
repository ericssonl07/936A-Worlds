#pragma once
#ifndef PATH_HPP
#define PATH_HPP

#include <matrix.hpp>
#include <vector>
#include <fstream>

/**
 * @example
 * ```cpp
 * // Example usage of the Coordinate2D struct:
 * Coordinate2D point1(3.0, 4.0); // Create a Coordinate2D object with x=3.0 and y=4.0
 * Coordinate2D point2 = point1 + Coordinate2D(1.0, 2.0); // Add two Coordinate2D objects
 * Coordinate2D point3 = point1 - point2; // Subtract two Coordinate2D objects
 * Coordinate2D point4 = point1 * 2.0; // Multiply a Coordinate2D object by a scalar
 * Coordinate2D point5 = point1 / 2.0; // Divide a Coordinate2D object by a scalar
 * std::cout << point1 << std::endl; // Print a Coordinate2D object
 * 
 * // Example usage of the CubicExpression struct:
 * CubicExpression cubic1(1.0, 2.0, 3.0, 4.0); // Create a CubicExpression object
 * std::cout << cubic1 << std::endl; // Print a CubicExpression object
 * 
 * // Example usage of the CubicSpline class:
 * CubicSpline spline({0.0, 1.0, 2.0}, {0.0, 1.0, 4.0}); // Create a CubicSpline object
 * std::vector<double> values = {0.5, 1.5, 2.5}; // Values to evaluate the spline
 * std::vector<double> results = spline(values); // Evaluate the spline at the given values
 * for (double result : results) {
 *     std::cout << result << " "; // Print the results
 * }
 * 
 * // Example usage of Path class:
 * std::vector<double> x_values = {0.0, 1.0, 2.0, 3.0};
 * std::vector<double> y_values = {0.0, 1.0, 4.0, 9.0};
 * Path path(x_values, y_values); // Create a Path object with given x and y values
 * path.serialize("path.txt"); // Serialize the path to a file
 * Path loaded_path("path.txt"); // Load the path from a file
 * Path path2({{0.0, 0.0}, {1.0, 1.0}, {2.0, 4.0}, {3.0, 9.0}}); // Create a Path object with given points
 * ```
 */

/**
 * @struct Coordinate2D
 * @file path.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This struct represents a 2D coordinate point with x and y values.
 * @example
 * ```cpp
 * // Example usage of Coordinate2D struct:
 * Coordinate2D point1(3.0, 4.0); // Create a Coordinate2D object with x=3.0 and y=4.0
 * Coordinate2D point2 = point1 + Coordinate2D(1.0, 2.0); // Add two Coordinate2D objects
 * Coordinate2D point3 = point1 - point2; // Subtract two Coordinate2D objects
 * Coordinate2D point4 = point1 * 2.0; // Multiply a Coordinate2D object by a scalar
 * Coordinate2D point5 = point1 / 2.0; // Divide a Coordinate2D object by a scalar
 * std::cout << point1 << std::endl; // Print a Coordinate2D object
 * ```
 */
struct Coordinate2D {
/**
 * @publicsection
 */

    /**
     * @public x
     * @brief X-coordinate of the point
     */
    double x;

    /**
     * @public y
     * @brief Y-coordinate of the point
     */
    double y;

    /**
     * @public constructor
     * @brief Default constructor for Coordinate2D
     * @details Initializes the x and y coordinates to 0.0.
     */
    Coordinate2D();

    /**
     * @public constructor
     * @brief Constructor for Coordinate2D
     * @param x X-coordinate of the point
     * @param y Y-coordinate of the point
     * @details Initializes the x and y coordinates to the specified values.
     */
    Coordinate2D(double x, double y);

    /**
     * @public constructor
     * @brief Constructor for Coordinate2D
     * @param point A pair of doubles representing the x and y coordinates
     * @details Initializes the x and y coordinates to the specified values.
     */
    Coordinate2D(std::pair<double, double> point);

    /**
     * @public constructor
     * @brief Copy constructor for Coordinate2D
     * @param other The Coordinate2D object to copy from
     * @details Initializes the x and y coordinates to the values of the specified object.
     */
    Coordinate2D(const Coordinate2D& other) = default;

    /**
     * @public constructor
     * @brief Move constructor for Coordinate2D
     * @param other The Coordinate2D object to move from
     * @details Initializes the x and y coordinates to the values of the specified object.
     */
    Coordinate2D(Coordinate2D&& other) = default;

    /**
     * @public assignment operator
     * @brief Assignment operator for Coordinate2D
     * @param other The Coordinate2D object to assign from
     * @details Assigns the x and y coordinates of the specified object to this object.
     */
    Coordinate2D& operator= (const Coordinate2D& other) = default;

    /**
     * @public assignment operator
     * @brief Move assignment operator for Coordinate2D
     * @param other The Coordinate2D object to move from
     * @details Assigns the x and y coordinates of the specified object to this object.
     */
    Coordinate2D& operator= (Coordinate2D&& other) = default;

    /**
     * @public destructor
     * @brief Destructor for Coordinate2D
     * @details Cleans up the resources used by the Coordinate2D object.
     */
    ~Coordinate2D() = default;

    /**
     * @public operator+
     * @brief Addition operator for Coordinate2D
     * @param other The Coordinate2D object to add to this object
     * @details Adds the x and y coordinates of the specified object to this object.
     * @returns A new Coordinate2D object with the sum of the coordinates.
     */
    Coordinate2D operator+ (const Coordinate2D& other);

    /**
     * @public operator-
     * @brief Subtraction operator for Coordinate2D
     * @param other The Coordinate2D object to subtract from this object
     * @details Subtracts the x and y coordinates of the specified object from this object.
     * @returns A new Coordinate2D object with the difference of the coordinates.
     */
    Coordinate2D operator- (const Coordinate2D& other);

    /**
     * @public operator*
     * @brief Multiplication operator for Coordinate2D
     * @param scalar The scalar value to multiply this object by
     * @details Multiplies the x and y coordinates of this object by the specified scalar.
     * @returns A new Coordinate2D object with the product of the coordinates.
     */
    Coordinate2D operator* (double scalar);

    /**
     * @public operator/
     * @brief Division operator for Coordinate2D
     * @param scalar The scalar value to divide this object by
     * @details Divides the x and y coordinates of this object by the specified scalar.
     * @returns A new Coordinate2D object with the quotient of the coordinates.
     */
    Coordinate2D operator/ (double scalar);

    /**
     * @public operator*(double, Coordinate2D)
     * @brief Multiplication operator for Coordinate2D
     * @param scalar The scalar value to multiply this object by
     * @param point The Coordinate2D object to multiply
     * @details Multiplies the x and y coordinates of this object by the specified scalar.
     * @returns A new Coordinate2D object with the product of the coordinates.
     * @relatesalso Coordinate2D
     */
    friend Coordinate2D operator* (double scalar, const Coordinate2D& point);

    /**
     * @public operator<<
     * @brief Output operator for Coordinate2D
     * @param os The output stream to write to
     * @param point The Coordinate2D object to write to the output stream
     * @details Writes the x and y coordinates of the specified object to the output stream.
     * @relatesalso Coordinate2D
     * @returns The output stream
     */
    friend std::ostream& operator<< (std::ostream& os, const Coordinate2D& point);
};

/**
 * @struct CubicExpression
 * @file path.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This struct represents a cubic expression with coefficients a, b, c, and d: y = ax^{3} + bx^{2} + cx + d.
 * @example
 * ```cpp
 * // Example usage of CubicExpression struct:
 * CubicExpression expression(1.0, -2.0, 3.0, -4.0); // y = x^{3} - 2x^{2} + 3x - 4
 * ```
 */
struct CubicExpression {
/**
 * @publicsection
 */

    /**
     * @public a
     * @brief Coefficient of x^{3}
     */
    double a;

    /**
     * @public b
     * @brief Coefficient of x^{2}
     */
    double b;

    /**
     * @public c
     * @brief Coefficient of x
     */
    double c;

    /**
     * @public d
     * @brief Constant term
     */
    double d;

    /**
     * @public constructor
     * @brief Default constructor for CubicExpression
     * @details Initializes the coefficients a, b, c, and d to 0.0.
     */
    CubicExpression() = default;

    /**
     * @public constructor
     * @brief Constructor for CubicExpression
     * @param a Coefficient of x^{3}
     * @param b Coefficient of x^{2}
     * @param c Coefficient of x
     * @param d Constant term
     * @details Initializes the coefficients a, b, c, and d to the specified values.
     */
    CubicExpression(double a, double b, double c, double d);

    /**
     * @public constructor
     * @brief Constructor for CubicExpression
     * @param coefficients A vector of coefficients [a, b, c, d]
     * @details Initializes the coefficients a, b, c, and d to the specified values.
     */
    CubicExpression(const CubicExpression& other) = default;

    /**
     * @public constructor
     * @brief Move constructor for CubicExpression
     * @param other The CubicExpression object to move from
     * @details Initializes the coefficients a, b, c, and d to the values of the specified object.
     */
    CubicExpression(CubicExpression&& other) = default;

    /**
     * @public assignment operator
     * @brief Assignment operator for CubicExpression
     * @param other The CubicExpression object to assign from
     * @details Assigns the coefficients a, b, c, and d of the specified object to this object.
     */
    CubicExpression& operator= (const CubicExpression& other) = default;

    /**
     * @public assignment operator
     * @brief Move assignment operator for CubicExpression
     * @param other The CubicExpression object to move from
     * @details Assigns the coefficients a, b, c, and d of the specified object to this object.
     */
    CubicExpression& operator= (CubicExpression&& other) = default;

    /**
     * @public destructor
     * @brief Destructor for CubicExpression
     * @details Cleans up the resources used by the CubicExpression object.
     */
    ~CubicExpression() = default;

    /**
     * @public operator()
     * @brief Evaluate the cubic expression at a given x value
     * @param x The x value to evaluate the expression at
     * @details Evaluates the cubic expression at the specified x value.
     * @relatesalso CubicExpression
     * @returns The result of the evaluation.
     */
    friend std::ostream& operator<< (std::ostream& os, const CubicExpression& expression);
};

/**
 * @class CubicSpline
 * @file path.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This class represents a cubic spline interpolation for a set of points.
 * It provides methods for constructing the spline and evaluating it at given points.
 * @details
 * @note
 * Cubic Spline interpolation is a method of constructing a smooth curve through a set of points using piecewise cubic polynomials.
 * The resulting curve is continuous and has continuous first and second derivatives (C^{2} continuity). This is achieved by
 * solving a system of linear equations (for n+1 points, n cubic polynomials giving 4n equations and 4n unknowns).
 * @example
 * ```cpp
 * // Example usage of CubicSpline class:
 * std::vector<double> x_values = {0.0, 1.0, 2.0, 3.0};
 * std::vector<double> y_values = {0.0, 1.0, 4.0, 9.0};
 * CubicSpline spline(x_values, y_values); // Create a CubicSpline object with given x and y values
 * std::vector<double> interpolated_values = spline({0.5, 1.5, 2.5}); // Evaluate the spline at given points
 * ```
 * @see
 * CubicExpression
 * 
 * Matrix
 * 
 * Path
 * 
 * Pursuit
 */
class CubicSpline {
/**
 * @privatesection
 */
    friend class Path;

    /**
     * @private operations
     * @brief Matrix of operations for representing the cubic spline system of equations
     */
    Matrix operations;

    /**
     * @private coefficients
     * @brief Vector of cubic expressions representing the coefficients of the spline
     */
    std::vector<CubicExpression> coefficients;

    /**
     * @private piecewise_boundaries
     * @brief Vector of piecewise boundaries for the cubic spline
     * @details The piecewise boundaries are the x-coordinates of the points used to construct the spline.
     */
    std::vector<double> piecewise_boundaries;

    /**
     * @private linspace
     * @brief Generate a linearly spaced vector of points
     * @param start The starting value of the range
     * @param end The ending value of the range
     * @param point_count The number of points to generate
     * @details This method generates a linearly spaced vector of points between the specified start and end values.
     * @returns A vector of doubles representing the linearly spaced points.
     */
    static std::vector<double> linspace(double start, double end, int point_count);

/**
 * @publicsection
 */
public:

    /**
     * @public get_segment
     * @brief Get the index of the segment for a given x value
     * @param of The x value to find the segment for
     * @details This method finds the index of the segment in which the specified x value lies.
     * @returns The index of the segment.
     */
    int get_segment(double of);

    /**
     * @public constructor
     * @brief Constructor for CubicSpline
     * @param x_values The x-coordinates of the points used to construct the spline
     * @param y_values The y-coordinates of the points used to construct the spline
     * @details This constructor initializes the cubic spline with the specified x and y values.
     * It constructs the spline by solving the system of equations for the cubic polynomials.
     * @throws std::logic_error if the x and y values are not of the same size
     * @throws std::logic_error if the x values are not strictly increasing
     * @throws std::logic_error if the x values are not unique
     */
    CubicSpline(std::vector<double> x_values, std::vector<double> y_values);

    /**
     * @public operator()
     * @brief Evaluate the cubic spline at a given x value
     * @param values The x values to evaluate the spline at
     * @details This method evaluates the cubic spline at the specified x values.
     * It returns a vector of y values corresponding to the input x values.
     * @returns A vector of doubles representing the evaluated y values.
     */
    std::vector<double> operator() (std::vector<double> values);
};

/**
 * @class Path
 * @file path.hpp
 * @author @ericssonl07
 * @date 2025-04-20
 * @brief
 * This class represents a path for a robot to follow, defined by a set of points in 2D space.
 * @details
 * The Path class is necessarily a wrapper around the CubicSpline class, providing an interface for constructing linear approximations
 * of a smooth curve given through a set of points.
 * The class provides methods for constructing the path from a set of points, evaluating the path at given points, and serializing the path to a file
 * for later use to avoid runtime overhead.
 * @note
 * Cubic spline interpolation uses piecewise cubic polynomials to create a smooth curve through a set of points. Importantly, a cubic
 * spline is a FUNCTION, meaning that it is a single-valued function for each x-coordinate. In order to create a generic path in 2D space
 * (which is almost certainly not a function and rather a relation), we need to use a parametric representation. The class achieves this
 * by generating a uniformly spaced set of time points along the path, which can be used to create a smooth spline for x and y independently.
 * @example
 * ```cpp
 * // Example usage of Path class:
 * std::vector<double> x_values = {0.0, 1.0, 2.0, 3.0};
 * std::vector<double> y_values = {0.0, 1.0, 4.0, 9.0};
 * Path path(x_values, y_values); // Create a Path object with given x and y values
 * path.serialize("path.txt"); // Serialize the path to a file
 * Path loaded_path("path.txt"); // Load the path from a file
 * Path path2({{0.0, 0.0}, {1.0, 1.0}, {2.0, 4.0}, {3.0, 9.0}}); // Create a Path object with given points
 * ```
 * @see
 * CubicSpline
 * 
 * Pursuit
 */
class Path {
/**
 * @privatesection
 */
    friend class Pursuit;

    friend class Chassis;

    friend int blue_ringrush();

    /**
     * @private points
     * @brief Vector of points representing the linear path approximation
     */
    std::vector<Coordinate2D> points;

    /**
     * @private construct_path
     * @brief Construct the path from a set of x and y values
     * @param x The x-coordinates of the points used to construct the path
     * @param y The y-coordinates of the points used to construct the path
     * @param point_count The number of points to generate
     * @details This method generates a set of points along the path using cubic spline interpolation.
     * @note We use CubicSpline::linspace to generate a set of uniform parameter values (think "time") along the path.
     */
    void construct_path(std::vector<double> x, std::vector<double> y, int point_count);

/**
 * @publicsection
 */
public:

    /**
     * @public operator[]
     * @brief Access a point in the path
     * @param idx The index of the point to access
     * @details This method returns the point at the specified index in the path.
     * @note This method does not perform bounds checking, so it is the user's responsibility to ensure that the index is valid.
     * @returns A reference to the specified point in the path.
     */
    Coordinate2D operator [] (int idx);

    /**
     * @public size
     * @brief Get the number of points in the path
     * @returns The number of points in the path.
     */
    std::size_t size();

    /**
     * @public constructor
     * @brief Default constructor for Path
     * @details Initializes an empty path with no points.
     */
    Path();

    /**
     * @public constructor
     * @brief Constructor for Path
     * @param x The x-coordinates of the points used to construct the path
     * @param y The y-coordinates of the points used to construct the path
     * @param point_count The number of points to generate. If -1, the number of points is determined by size(points) * 10.
     * @details This constructor initializes the path with the specified x and y values.
     * It constructs the path by generating a set of points along the path using cubic spline interpolation.
     * @throws std::logic_error if the x and y values are not of the same size
     */
    Path(std::vector<double> x, std::vector<double> y, int point_count = -1);

    /**
     * @public constructor
     * @brief Constructor for Path
     * @param points A vector of points representing the path
     * @param point_count The number of points to generate. If -1, the number of points is determined by size(points) * 10.
     * @details This constructor initializes the path with the specified points.
     * It constructs the path by generating a set of points along the path using cubic spline interpolation.
     */
    Path(std::vector<std::pair<double, double>> points, int point_count = -1);

    /**
     * @public constructor
     * @brief Constructor for Path
     * @param filename The name of the file to load the path from
     * @details This constructor initializes the path by loading the points from the specified file.
     * The file should contain the points in the ".path" format.
     * @attention It is the user's responsibility to ensure that the file exists and is in the correct format.
     * Attempting to load a nonexistent or manually created/modified file will result in undefined behavior.
     * The file must be a ".path" file.
     * @throws std::logic_error if the path file is not a ".path" file
     */
    Path(std::string filename);

    /**
     * @public constructor
     * @brief Copy constructor for Path
     * @param other The Path object to copy from
     * @details Initializes the path by copying the points from the specified object.
     */
    Path(const Path& other) = default;

    /**
     * @public constructor
     * @brief Move constructor for Path
     * @param other The Path object to move from
     * @details Initializes the path by moving the points from the specified object.
     */
    Path(Path&& other) = default;

    /**
     * @public assignment operator
     * @brief Assignment operator for Path
     * @param other The Path object to assign from
     * @details Assigns the points of the specified object to this object.
     */
    Path& operator= (const Path& other) = default;

    /**
     * @public assignment operator
     * @brief Move assignment operator for Path
     * @param other The Path object to move from
     * @details Assigns the points of the specified object to this object.
     */
    Path& operator= (Path&& other) = default;

    /**
     * @public serialize
     * @brief Serialize the path to a file
     * @param filename The name of the file to save the path to- must end with ".path"
     * @details This method saves the points of the path to the specified file in the ".path" format.
     * The file will contain the points in a human-readable format, with each point on a new line.
     * The format is as follows:
     * ```
     * P x1 y1
     * P x2 y2
     * ...
     * P xn yn
     * E
     * ```
     * @note The file format is:
     * ```
     * P x1 y1
     * P x2 y2
     * ...
     * P xn yn
     * E
     * ```
     * @attention The file must be a ".path" file.
     * @throws std::logic_error if the filename is not a ".path" file
     */
    void serialize(std::string filename);
};

#endif // #ifndef PATH_HPP