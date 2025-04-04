#pragma once
#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <iostream>
#include <iomanip>
#include <utility>
#include <vector>
#include <cmath>
#include <map>

class Matrix {
    std::pair<int, int> matrix_dimensions;
    std::vector<std::vector<double>> internal_matrix_memory;
    int output_precision = 6;
    void swap_row(int idx_row_1, int idx_row_2);
    void row_operation(int idx_row, std::map<int, double> row_operations_map);
public:
    int set_output_precision(int precision);
    const std::pair<int, int> & dimensions();
    std::vector<double>& operator[] (int idx);
    double & at(int i, int j);
    Matrix();
    Matrix(Matrix& copy_from);
    Matrix(Matrix&& move_from);
    Matrix(std::pair<int, int> matrix_dimensions, double flood_value);
    Matrix(std::pair<int, int> matrix_dimensions, double * fill_values_unsafe_ptr);
    Matrix(std::vector<std::vector<double>> data_values);
    Matrix& operator= (Matrix& copy_from);
    Matrix& operator= (Matrix&& move_from);
    Matrix operator- ();
    Matrix operator+ (Matrix& addend);
    Matrix operator- (Matrix& subtrahend);
    Matrix operator* (Matrix& multiplicand);
    Matrix augment(Matrix& augmentation_values, int axis = 1);
    Matrix transpose();
    Matrix rref();
    Matrix inverse();
    Matrix operator* (double scalar);
    Matrix operator/ (double scalar);
    double determinant();
    friend std::ostream& operator << (std::ostream& os, Matrix& matrix);
    friend std::ostream& operator << (std::ostream& os, Matrix&& matrix);
};

Matrix identity(int n_dimensions);
Matrix zeros(int row_count, int column_count);
Matrix zeros_like(Matrix& reference_matrix);
Matrix ones(int row_count, int column_count);
Matrix ones_like(Matrix& reference_matrix);

#endif // #ifndef MATRIX_HPP