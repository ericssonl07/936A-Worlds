#include <matrix.hpp>
#include <iostream>
#include <iomanip>
#include <utility>
#include <vector>
#include <cmath>
#include <map>

void Matrix::swap_row(int idx_row_1, int idx_row_2) {
    if (idx_row_1 == idx_row_2) {
        return;
    }
    std::swap(internal_matrix_memory[idx_row_1], internal_matrix_memory[idx_row_2]);
}

void Matrix::row_operation(int idx_row, std::map<int, double> row_operations_map) {
    std::vector<double> result(matrix_dimensions.second, 0);
    for (auto [row, scalar] : row_operations_map) {
        for (int i = 0; i < matrix_dimensions.second; ++i) {
            result[i] += internal_matrix_memory[row][i] * scalar;
        }
    }
    internal_matrix_memory[idx_row] = result;
}

int Matrix::set_output_precision(int precision) {
    output_precision = precision;
    return output_precision;
}

const std::pair<int, int> & Matrix::dimensions() {
    return matrix_dimensions;
}

std::vector<double>& Matrix::operator[] (int idx) {
    return internal_matrix_memory[idx];
}

double & Matrix::at(int i, int j) {
    if (i < 0 || i >= matrix_dimensions.first || j < 0 || j >= matrix_dimensions.second) {
        std::cerr << "Index out of bounds\n";
        throw std::out_of_range("Index out of bounds");
    }
    return internal_matrix_memory[i][j];
}

Matrix::Matrix() {
    matrix_dimensions = std::make_pair(0, 0);
    internal_matrix_memory = std::vector<std::vector<double>>();
}

Matrix::Matrix(Matrix& copy_from) {
    matrix_dimensions = copy_from.matrix_dimensions;
    internal_matrix_memory = copy_from.internal_matrix_memory;
}

Matrix::Matrix(Matrix&& move_from) {
    matrix_dimensions = std::move(move_from.matrix_dimensions);
    internal_matrix_memory = std::move(move_from.internal_matrix_memory);
    move_from.matrix_dimensions = std::make_pair(0, 0);
    move_from.internal_matrix_memory = std::vector<std::vector<double>>();
}

Matrix::Matrix(std::pair<int, int> matrix_dimensions, double flood_value) {
    Matrix::matrix_dimensions = matrix_dimensions;
    internal_matrix_memory.assign(matrix_dimensions.first, std::vector<double>(matrix_dimensions.second, flood_value));
}

Matrix::Matrix(std::pair<int, int> matrix_dimensions, double * fill_values_unsafe_ptr) {
    Matrix::matrix_dimensions = matrix_dimensions;
    internal_matrix_memory.assign(matrix_dimensions.first, std::vector<double>(matrix_dimensions.second, 0.0));
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            int index = i * matrix_dimensions.second + j;
            internal_matrix_memory[i][j] = fill_values_unsafe_ptr[index];
        }
    }
}

Matrix::Matrix(std::vector<std::vector<double>> data_values) {
    int homogeneous_length = data_values[0].size();
    for (int i = 0; i < data_values.size(); ++i) {
        if (data_values[i].size() != homogeneous_length) {
            std::cerr << "Inhomogeneous vector shapes\n";
            throw std::logic_error("Inhomogeneous vector shapes");
        }
    }
    matrix_dimensions.first = data_values.size();
    matrix_dimensions.second = homogeneous_length;
    internal_matrix_memory = data_values;
}

Matrix& Matrix::operator= (Matrix& copy_from) {
    matrix_dimensions = copy_from.matrix_dimensions;
    internal_matrix_memory = copy_from.internal_matrix_memory;
    return *this;
}

Matrix& Matrix::operator= (Matrix&& move_from) {
    matrix_dimensions = std::move(move_from.matrix_dimensions);
    internal_matrix_memory = std::move(move_from.internal_matrix_memory);
    return *this;
}

Matrix Matrix::operator- () {
    Matrix result(*this);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[i][j] = -result[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator+ (Matrix& addend) {
    if (matrix_dimensions != addend.matrix_dimensions) {
        std::cerr << "Dimension mismatch\n";
        throw std::logic_error("Dimension mismatch");
    }
    Matrix result = zeros_like(*this);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[i][j] = internal_matrix_memory[i][j] + addend[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator- (Matrix& subtrahend) {
    if (matrix_dimensions != subtrahend.matrix_dimensions) {
        std::cerr << "Dimension mismatch\n";
        throw std::logic_error("Dimension mismatch");
    }
    Matrix result = zeros_like(*this);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[i][j] = internal_matrix_memory[i][j] - subtrahend[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator* (Matrix& multiplicand) {
    if (matrix_dimensions.second != multiplicand.matrix_dimensions.first) {
        std::cerr << "Dimension mismatch\n";
        throw std::logic_error("Dimension mismatch");
    }
    Matrix result = zeros(matrix_dimensions.first, multiplicand.matrix_dimensions.second);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < multiplicand.matrix_dimensions.second; ++j) {
            for (int k = 0; k < matrix_dimensions.second; ++k) {
                result[i][j] += internal_matrix_memory[i][k] * multiplicand[k][j];
            }
        }
    }
    return result;
}

Matrix Matrix::augment(Matrix& augmentation_values, int axis) {
    if (axis != 0 and axis != 1) {
        std::cerr << "Invalid axis\n";
        throw std::logic_error("Invalid axis");
    }
    if (axis == 1) {
        if (matrix_dimensions.first != augmentation_values.matrix_dimensions.first) {
            std::cerr << "Dimension mismatch\n";
            throw std::logic_error("Dimension mismatch");
        }
        Matrix result = zeros(matrix_dimensions.first, matrix_dimensions.second + augmentation_values.matrix_dimensions.second);
        for (int i = 0; i < matrix_dimensions.first; ++i) {
            for (int j = 0; j < matrix_dimensions.second; ++j) {
                result[i][j] = internal_matrix_memory[i][j];
            }
            for (int j = 0; j < augmentation_values.matrix_dimensions.second; ++j) {
                result[i][j + matrix_dimensions.second] = augmentation_values[i][j];
            }
        }
        return result;
    }
    if (matrix_dimensions.second != augmentation_values.matrix_dimensions.second) {
        std::cerr << "Dimension mismatch\n";
        throw std::logic_error("Dimension mismatch");
    }
    Matrix result = zeros(matrix_dimensions.first + augmentation_values.matrix_dimensions.first, matrix_dimensions.second);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[i][j] = internal_matrix_memory[i][j];
        }
        for (int j = 0; j < augmentation_values.matrix_dimensions.second; ++j) {
            result[i + matrix_dimensions.first][j] = augmentation_values[i][j];
        }
    }
    return result;
}

Matrix Matrix::transpose() {
    Matrix result = zeros(matrix_dimensions.second, matrix_dimensions.first);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[j][i] = internal_matrix_memory[i][j];
        }
    }
    return result;
}

Matrix Matrix::rref() {
    Matrix result(*this);
    int pivot_row = 0, pivot_column = 0;
    bool zero = true;
    std::vector<std::pair<int, int>> pivot_points;
    while (pivot_row < matrix_dimensions.first and pivot_column < matrix_dimensions.second) {
        if (result[pivot_row][pivot_column] == 0) {
            zero = true;
            for (int i = pivot_row + 1; i < matrix_dimensions.first; ++i) {
                if (result[i][pivot_column] != 0) {
                    result.swap_row(pivot_row, i);
                    zero = false;
                    break;
                }
            }
            if (zero) {
                ++pivot_column;
                continue;
            }
        }
        pivot_points.push_back(std::make_pair(pivot_row, pivot_column));
        result.row_operation(pivot_row, {{pivot_row, 1 / result[pivot_row][pivot_column]}});
        for (int i = pivot_row + 1; i < matrix_dimensions.first; ++i) {
            if (result[i][pivot_column] != 0) {
                result.row_operation(i, {{i, 1}, {pivot_row, -result[i][pivot_column] / result[pivot_row][pivot_column]}});
            }
        }
        ++pivot_row;
        ++pivot_column;
    }
    for (std::vector<std::pair<int, int>>::reverse_iterator iterator = pivot_points.rbegin(); iterator != pivot_points.rend(); ++iterator) {
        std::pair<int, int> point = *iterator;
        pivot_row = point.first;
        pivot_column = point.second;
        result.row_operation(pivot_row, {{pivot_row, 1 / result[pivot_row][pivot_column]}});
        for (int i = pivot_row - 1; i >= 0; --i) {
            if (result[i][pivot_column] != 0) {
                result.row_operation(i, {{i, 1}, {pivot_row, -result[i][pivot_column] / result[pivot_row][pivot_column]}});
            }
        }
    }
    return result;
}

Matrix Matrix::inverse() {
    if (matrix_dimensions.first != matrix_dimensions.second) {
        std::cerr << "Nonsquare matrix does not have an inverse\n";
        throw std::logic_error("Nonsquare matrix does not have an inverse");
    }
    if (fabs(determinant()) < 1e-10) {
        std::cerr << "Singular matrix does not have an inverse\n";
        throw std::domain_error("Singular matrix does not have an inverse");
    }
    Matrix identity_augmentation = identity(matrix_dimensions.first);
    Matrix augmented_matrix = augment(identity_augmentation);
    Matrix reduced_matrix = augmented_matrix.rref();
    Matrix result = zeros_like(*this);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[i][j] = reduced_matrix[i][j + matrix_dimensions.second];
        }
    }
    return result;
}

Matrix Matrix::operator* (double scalar) {
    Matrix result(*this);
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[i][j] *= scalar;
        }
    }
    return result;
}

Matrix Matrix::operator/ (double scalar) {
    Matrix result(*this);
    double reciprocal = 1 / scalar;
    for (int i = 0; i < matrix_dimensions.first; ++i) {
        for (int j = 0; j < matrix_dimensions.second; ++j) {
            result[i][j] *= reciprocal;
        }
    }
    return result;
}

double Matrix::determinant() {
    if (matrix_dimensions.first != matrix_dimensions.second) {
        std::cerr << "Nonsquare matrix does not have a determinant\n";
        throw std::logic_error("Nonsquare matrix does not have a determinant");
    }
    Matrix result(*this);
    int pivot_row = 0, pivot_column = 0;
    int swaps_count = 0;
    bool zero = true;
    while (pivot_row < matrix_dimensions.first and pivot_column < matrix_dimensions.second) {
        if (result[pivot_row][pivot_column] == 0) {
            zero = true;
            for (int i = pivot_row + 1; i < matrix_dimensions.first; ++i) {
                if (result[i][pivot_column] != 0) {
                    ++swaps_count;
                    result.swap_row(pivot_row, i);
                    zero = false;
                    break;
                }
            }
            if (zero) {
                ++pivot_column;
                continue;
            }
        }
        for (int i = pivot_row + 1; i < matrix_dimensions.first; ++i) {
            if (result[i][pivot_column] != 0) {
                result.row_operation(i, {{i, 1}, {pivot_row, -result[i][pivot_column] / result[pivot_row][pivot_column]}});
            }
        }
        ++pivot_row;
        ++pivot_column;
    }
    double determinant_value = result[0][0];
    for (int i = 1; i < matrix_dimensions.first; ++i) {
        determinant_value *= result[i][i];
    }
    if (swaps_count % 2 == 1) {
        return -determinant_value;
    }
    return determinant_value;
}

std::ostream& operator<< (std::ostream& os, Matrix& matrix) {
    os << std::fixed << std::setprecision(matrix.output_precision);
    os << "Matrix shape (" << matrix.matrix_dimensions.first << ", " << matrix.matrix_dimensions.second << ")\n";
    for (auto row : matrix.internal_matrix_memory) {
        for (int j = 0; j < matrix.matrix_dimensions.second; ++j) {
            os << row[j] << (j == matrix.matrix_dimensions.second - 1 ? '\n' : ' ');
        }
    }
    return os;
}

std::ostream& operator<< (std::ostream& os, Matrix&& matrix) {
    os << std::fixed << std::setprecision(matrix.output_precision);
    os << "Matrix shape (" << matrix.matrix_dimensions.first << ", " << matrix.matrix_dimensions.second << ")\n";
    for (auto row : matrix.internal_matrix_memory) {
        for (int j = 0; j < matrix.matrix_dimensions.second; ++j) {
            os << row[j] << (j == matrix.matrix_dimensions.second - 1 ? '\n' : ' ');
        }
    }
    return os;
}

Matrix identity(int n_dimensions) {
    Matrix result(std::make_pair(n_dimensions, n_dimensions), 0.0);
    for (int i = 0; i < n_dimensions; ++i) {
        result[i][i] = 1.0;
    }
    return result;
}

Matrix zeros(int row_count, int column_count) {
    return Matrix(std::make_pair(row_count, column_count), 0.0);
}

Matrix zeros_like(Matrix& reference_matrix) {
    return Matrix(std::make_pair(reference_matrix.dimensions().first, reference_matrix.dimensions().second), 0.0);
}

Matrix ones(int row_count, int column_count) {
    return Matrix(std::make_pair(row_count, column_count), 1.0);
}

Matrix ones_like(Matrix& reference_matrix) {
    return Matrix(std::make_pair(reference_matrix.dimensions().first, reference_matrix.dimensions().second), 1.0);
}

// Last reviewed on Saturday, February 22nd, 2025