#pragma once
#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <iostream>
#include <iomanip>
#include <utility>
#include <vector>
#include <cmath>
#include <map>

/**
 * @class Matrix
 * @file matrix.hpp
 * @author @ericssonl07
 * @date 2025-04-19
 * @brief
 * This class represents a matrix and provides various operations for matrix manipulation, including addition, subtraction, multiplication, and inversion.
 * It also includes methods for row operations, transposition, and finding the reduced row echelon form (RREF) of the matrix.
 * The class is designed to be flexible and efficient, allowing for easy manipulation of matrices of different sizes and types.
 * The Matrix class is implemented using a 2D vector to store the matrix elements, and it provides methods for accessing and modifying individual elements.
 * The class also includes methods for setting the output precision for displaying matrix elements, as well as methods for creating identity matrices, zero matrices, and one matrices.
 * The class is designed to be used in a variety of applications, including linear algebra, numerical analysis, and scientific computing.
 * Here, the specific usage is for solving a Cubic Spline Interpolation problem, where the matrix is used to represent the system of equations that arise from the interpolation process.
 * @details
 * Most of the methods are implemented using standard algorithms for matrix operations, such as Gaussian elimination for finding the RREF and inverse of a matrix.
 * The class is NOT meant to be a complete implementation of all possible matrix operations, but rather a focused implementation for the specific use case of solving linear systems and performing matrix manipulations.
 * Additionally, it is NOT meant to be a high-performance implementation replacing libraries like Eigen or Armadillo, but rather a simple and easy-to-use implementation for this specific use case.
 * The class is designed to be easy to use and understand, with clear and concise method names and parameters.
 * The class also includes error handling for invalid matrix operations, such as attempting to add or multiply matrices of incompatible sizes.
 * @note
 * Matrix multiplication is implemented using the standard algorithm for matrix multiplication, which involves taking the dot product of rows and columns.
 * For a matrix A of size m x n and a matrix B of size n x p, the resulting matrix C will be of size m x p, where each element C[i][j] is computed as the sum of the products of the corresponding elements from row i of A and column j of B.
 * The complexity will be O(m * n * p). For square matrices, the complexity will be O(k^3) (m = n = p = k).
 * 
 * The inverse of a matrix is computed using Gaussian elimination, which involves transforming the matrix into its reduced row echelon form (RREF) and then applying back substitution to find the inverse.
 * The complexity of this operation is O(n^3) for an n x n matrix, and the class checks for singularity (non-invertibility) first by checking if the determinant is zero.
 * 
 * The determinant of the matrix is computed with Gaussian elimination as well, which involves transforming the matrix into an upper triangular form and then multiplying the diagonal elements while
 * tracking the number of row swaps made during the process. The determinant is zero if the matrix is singular (not invertible).
 * The complexity of this operation is also O(n^3) for an n x n matrix. This is more efficient than the Laplace expansion method, with complexity O(n!).
 * @example
 * ```cpp
 * // Example usage of the Matrix class:
 * Matrix A({{1, 2}, {3, 4}}); // Create a 2x2 matrix using nested list- dimensions must be the same
 * Matrix B({2, 2}, 4); // Create a 2x2 matrix filled with 4
 * Matrix C = A + B; // Add two matrices
 * Matrix D = A - B; // Subtract two matrices
 * Matrix E = A * B; // Multiply two matrices
 * Matrix F = A.transpose(); // Transpose a matrix
 * Matrix G = A.rref(); // Get the reduced row echelon form of a matrix
 * Matrix H = A.inverse(); // Get the inverse of a matrix
 * double det = A.determinant(); // Get the determinant of a matrix
 * Matrix I3 = identity(3); // Create a 3x3 identity matrix
 * Matrix Z2_2 = zeros(2, 2); // Create a 2x2 zero matrix
 * Matrix O2_2 = ones(2, 2); // Create a 2x2 one matrix
 * Matrix Z_like = zeros_like(A); // Create a zero matrix with the same dimensions as A
 * Matrix O_like = ones_like(A); // Create a one matrix with the same dimensions as A
 * std::cout << A << std::endl; // Print a matrix
 * ```
 */
class Matrix {

/**
 * @privatesection
 */
private:

    /**
     * @private matrix_dimensions
     * @brief Dimensions of the matrix (rows, columns)
     * @details The stored matrix has dimensions matrix_dimensions.first x matrix_dimensions.second.
     */
    std::pair<int, int> matrix_dimensions;

    /**
     * @private internal_matrix_memory
     * @brief Internal storage for the matrix elements
     * @details The matrix is stored as a 2D vector of doubles, where each element represents a value in the matrix.
     * 
     * internal_matrix_memory[i][j] represents the value at row i and column j.
     * @attention Indexing starts at 0, so internal_matrix_memory[0][0] is the first element of the matrix.
     * This contrasts with mathematical notation, where indexing starts at 1.
     */
    std::vector<std::vector<double>> internal_matrix_memory;

    /**
     * @private output_precision
     * @brief Output precision for displaying matrix elements
     * @details The output precision is set to 6 decimal places by default.
     * This can be changed using the set_output_precision method.
     */
    int output_precision = 6;

    /**
     * @private swap_row
     * @brief Swap two rows in the matrix
     * @param idx_row_1 Index of the first row to swap
     * @param idx_row_2 Index of the second row to swap
     * @details This method swaps the elements of the two specified rows in the matrix.
     */
    void swap_row(int idx_row_1, int idx_row_2);

    /**
     * @private row_operation
     * @brief Perform a row operation on the matrix
     * @param idx_row Index of the row to operate on
     * @param row_operations_map Map of column indices and their corresponding multipliers
     * @note
     * Let r=idx_row, and row_operations_map be a set of tuples (i, k) where i is unique. Let the rows be r_{0}, ..., r_{n-1}.
     * 
     * Then the operation performed is:
     * 
     * r_{r} := r_{r} + k_{0} * r_{i_{0}} + k_{1} * r_{i_{1}} + ... + k_{m} * r_{i_{m}}
     * 
     * The operation is performed in-place.
     */
    void row_operation(int idx_row, std::map<int, double> row_operations_map);

/**
 * @publicsection
 */
public:

    /**
     * @public set_output_precision
     * @brief Set the output precision for displaying matrix elements
     * @param precision Number of decimal places to display
     * @returns The new output precision
     */
    int set_output_precision(int precision);

    /**
     * @public dimensions
     * @brief Get the dimensions of the matrix
     * @returns A pair of integers representing the number of rows and columns in the matrix
     */
    const std::pair<int, int> & dimensions();

    /**
     * @public operator[]
     * @brief Access a row of the matrix without bounds checking- will encounter a runtime SIGSEGV error if the index is out of bounds
     * @param idx Index of the row to access
     * @returns A reference to the specified row of the matrix, which is a std::vector<double>
     */
    std::vector<double>& operator[] (int idx);

    /**
     * @public at
     * @brief Access an element of the matrix with bounds checking
     * @throws std::out_of_range if the indices are out of bounds
     * @param i Row index of the element to access
     * @param j Column index of the element to access
     * @returns A reference to the specified element of the matrix
     */
    double & at(int i, int j);

    /**
     * @public constructor
     * @brief Default constructor
     * @details Creates an empty matrix with dimensions (0, 0)
     */
    Matrix();

    /**
     * @public constructor
     * @brief Copy constructor
     * @param copy_from The matrix to copy from
     * @details Creates a new matrix by copying the elements of the specified matrix
     */
    Matrix(Matrix& copy_from);

    /**
     * @public constructor
     * @brief Move constructor
     * @param move_from The matrix to move from
     * @details Creates a new matrix by moving the elements of the specified matrix
     */
    Matrix(Matrix&& move_from);

    /**
     * @public constructor
     * @brief Constructor with dimensions and fill value
     * @param matrix_dimensions The dimensions of the matrix (rows, columns)
     * @param flood_value The value to fill the matrix with
     * @details Creates a new matrix with the specified dimensions and fills it with the specified value
     */
    Matrix(std::pair<int, int> matrix_dimensions, double flood_value);

    /**
     * @public constructor
     * @brief Constructor with dimensions and unsafe pointer
     * @param matrix_dimensions The dimensions of the matrix (rows, columns)
     * @param fill_values_unsafe_ptr A double array to fill the matrix with- unsafe if the pointer is not valid, or
     * if the array is not large enough to fill the matrix
     * @details Creates a new matrix with the specified dimensions and fills it with the values pointed to by the specified pointer
     */
    Matrix(std::pair<int, int> matrix_dimensions, double * fill_values_unsafe_ptr);

    /**
     * @public constructor
     * @brief Constructor using a nested list
     * @param data_values A nested list of doubles to fill the matrix with
     * @details Creates a new matrix with the specified dimensions and fills it with the values in the nested list
     * The dimensions of the matrix are determined by the size of the nested list.
     * @attention The vector of vectors must be homogeneous, so all inner vectors must have the same size.
     * @throws std::logic_error if the inner vectors are not the same size
     */
    Matrix(std::vector<std::vector<double>> data_values);

    /**
     * @public operator=
     * @brief Copy assignment operator
     * @param copy_from The matrix to copy from
     * @details Assigns the elements of the specified matrix to this matrix, making a deep copy
     */
    Matrix& operator= (Matrix& copy_from);

    /**
     * @public operator=
     * @brief Move assignment operator
     * @param move_from The matrix to move from
     * @details Assigns the elements of the specified matrix to this matrix, leaving move_from in a valid but unspecified state
     */
    Matrix& operator= (Matrix&& move_from);

    /**
     * @public operator-
     * @brief Negation operator
     * @details Returns a new matrix that is the negation of this matrix
     * @returns A new matrix that is the negation of this matrix
     */
    Matrix operator- ();

    /**
     * @public operator+
     * @brief Addition operator
     * @param addend The matrix to add to this matrix
     * @details Returns a new matrix that is the sum of this matrix and the specified matrix
     * @throws std::logic_error if the dimensions of the matrices do not match
     */
    Matrix operator+ (Matrix& addend);

    /**
     * @public operator-
     * @brief Subtraction operator
     * @param subtrahend The matrix to subtract from this matrix
     * @details Returns a new matrix that is the difference of this matrix and the specified matrix
     * @throws std::logic_error if the dimensions of the matrices do not match
     */
    Matrix operator- (Matrix& subtrahend);

    /**
     * @public operator*
     * @brief Multiplication operator
     * @param multiplicand The matrix to multiply this matrix by
     * @details Returns a new matrix that is the product of this matrix and the specified matrix
     * @throws std::logic_error if the dimensions of the matrices do not match
     */
    Matrix operator* (Matrix& multiplicand);

    /**
     * @public augment
     * @brief Augment the matrix with another matrix
     * @param augmentation_values The matrix to augment this matrix with
     * @param axis The axis to augment along (0 for "vertical augmentation", 1 for "horizontal augmentation").
     * Defaults to 1 (horizontal augmentation, as in Gaussian elimination).
     * @details Returns a new matrix that is the result of augmenting this matrix with the specified matrix
     * @throws std::logic_error if the dimensions of the matrices do not match
     * @returns A new matrix that is the result of augmenting this matrix with the specified matrix
     * @note
     * The augmentation is not performed in-place.
     */
    Matrix augment(Matrix& augmentation_values, int axis = 1);

    /**
     * @public transpose
     * @brief Transpose the matrix
     * @details Returns a new matrix that is the transpose of this matrix
     * @returns A new matrix that is the transpose of this matrix
     */
    Matrix transpose();

    /**
     * @public inverse
     * @brief Inverse the matrix
     * @details Returns a new matrix that is the inverse of this matrix
     * @returns The reduced row echelon form of the matrix
     */
    Matrix rref();

    /**
     * @public inverse
     * @brief Inverse the matrix
     * @details Returns a new matrix that is the inverse of this matrix
     * @throws std::logic_error if the matrix is not square or if it is singular
     * @returns A new matrix that is the inverse of this matrix, if it exists
     */
    Matrix inverse();

    /**
     * @public operator*
     * @brief Scalar multiplication operator
     * @param scalar The scalar to multiply this matrix by
     * @details Returns a new matrix that is the product of this matrix and the specified scalar
     * @returns A new matrix that is the product of this matrix and the specified scalar
     */
    Matrix operator* (double scalar);

    /**
     * @public operator/
     * @brief Scalar division operator
     * @param scalar The scalar to divide this matrix by
     * @details Returns a new matrix that is the quotient of this matrix and the specified scalar
     * @returns A new matrix that is the quotient of this matrix and the specified scalar
     */
    Matrix operator/ (double scalar);

    /**
     * @public determinant
     * @brief Calculate the determinant of the matrix
     * @details Returns the determinant of this matrix
     * @throws std::logic_error if the matrix is not square
     * @returns The determinant of this matrix, if it exists
     */
    double determinant();

    /**
     * @public operator<<
     * @brief Output operator
     * @param os The output stream to write to
     * @param matrix The matrix to write to the output stream
     * @details Writes the matrix to the specified output stream in a human-readable format
     * @returns The output stream
     */
    friend std::ostream& operator << (std::ostream& os, Matrix& matrix);

    /**
     * @public operator<<
     * @brief Output operator
     * @param os The output stream to write to
     * @param matrix The matrix to write to the output stream
     * @details Writes the matrix to the specified output stream in a human-readable format
     * @returns The output stream
     */
    friend std::ostream& operator << (std::ostream& os, Matrix&& matrix);
};

/**
 * @public identity
 * @brief Create an identity matrix of the specified size
 * @param n_dimensions The size of the identity matrix (n x n)
 * @details Returns a new identity matrix of the specified size
 * @returns A new identity matrix of the specified size
 * @relatesalso Matrix
 */
Matrix identity(int n_dimensions);

/**
 * @public zeros
 * @brief Create a zero matrix of the specified size
 * @param row_count The number of rows in the zero matrix
 * @param column_count The number of columns in the zero matrix
 * @details Returns a new zero matrix of the specified size
 * @returns A new zero matrix of the specified size
 * @relatesalso Matrix
 */
Matrix zeros(int row_count, int column_count);

/**
 * @public zeros_like
 * @brief Create a zero matrix with the same dimensions as the specified matrix
 * @param reference_matrix The matrix to copy the dimensions from
 * @details Returns a new zero matrix with the same dimensions as the specified matrix
 * @returns A new zero matrix with the same dimensions as the specified matrix
 * @relatesalso Matrix
 */
Matrix zeros_like(Matrix& reference_matrix);

/**
 * @public ones
 * @brief Create a one matrix of the specified size
 * @param row_count The number of rows in the one matrix
 * @param column_count The number of columns in the one matrix
 * @details Returns a new one matrix of the specified size
 * @returns A new one matrix of the specified size
 * @relatesalso Matrix
 */
Matrix ones(int row_count, int column_count);

/**
 * @public ones_like
 * @brief Create a one matrix with the same dimensions as the specified matrix
 * @param reference_matrix The matrix to copy the dimensions from
 * @details Returns a new one matrix with the same dimensions as the specified matrix
 * @returns A new one matrix with the same dimensions as the specified matrix
 * @relatesalso Matrix
 */
Matrix ones_like(Matrix& reference_matrix);

#endif // #ifndef MATRIX_HPP