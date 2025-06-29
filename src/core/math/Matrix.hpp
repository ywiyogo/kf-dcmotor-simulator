#pragma once

#include <array>
#include <vector>
#include <concepts>
#include <algorithm>
#include <execution>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <format>
#include <span>
#include <cmath>
#include <iostream>
#include <limits>

#ifdef HAS_PARALLEL_STL
#include <execution>
#endif

namespace kf::math {

/**
 * @brief Modern C++23 matrix class with SIMD-friendly operations
 *
 * Optimized for small matrices used in Kalman filtering with support
 * for parallel execution policies and vectorized operations.
 *
 * @tparam T Numeric type (float, double)
 * @tparam Rows Number of rows (compile-time)
 * @tparam Cols Number of columns (compile-time)
 */
template<std::floating_point T, std::size_t Rows, std::size_t Cols>
class Matrix {
public:
    // ============================================================================
    // Type Definitions and Constants
    // ============================================================================

    /** @brief The underlying arithmetic type used for matrix elements */
    using value_type = T;

    /** @brief Size type for indexing operations */
    using size_type = std::size_t;

    /** @brief Reference type for matrix elements */
    using reference = T&;

    /** @brief Const reference type for matrix elements */
    using const_reference = const T&;

    /** @brief Iterator type for traversing matrix elements */
    using iterator = typename std::array<T, Rows * Cols>::iterator;

    /** @brief Const iterator type for read-only traversal */
    using const_iterator = typename std::array<T, Rows * Cols>::const_iterator;

    /** @brief Number of rows in the matrix (compile-time constant) */
    static constexpr size_type rows = Rows;

    /** @brief Number of columns in the matrix (compile-time constant) */
    static constexpr size_type cols = Cols;

    /** @brief Total number of elements in the matrix (rows × cols) */
    static constexpr size_type size = Rows * Cols;

    /** @brief Threshold for switching from sequential to parallel execution */
    static constexpr size_type parallel_threshold = 1000;

    /** @brief Threshold for GPU acceleration (when available) */
    static constexpr size_type gpu_threshold = 10000;

    // ============================================================================
    // Constructors and Initialization
    // ============================================================================

    /**
     * @brief Default constructor - initializes all elements to zero
     *
     * Creates a matrix with all elements initialized to the default value of T (usually 0).
     * This is a constexpr constructor, allowing compile-time matrix initialization.
     *
     * @note This constructor is noexcept and can be used in constexpr contexts
     */
    constexpr Matrix() noexcept : data_{} {}

    /**
     * @brief Fill constructor - initializes all elements to a specific value
     *
     * @param value The value to fill the entire matrix with
     *
     * Creates a matrix where every element is set to the specified value.
     * Useful for creating identity matrices (when value=1) or zero matrices (when value=0).
     *
     * @note Explicit to prevent accidental implicit conversions
     * @note constexpr for compile-time evaluation when possible
     */
    explicit constexpr Matrix(T value) noexcept {
        data_.fill(value);
    }

    constexpr Matrix(std::initializer_list<std::initializer_list<T>> init) {
        if (init.size() != Rows) {
            throw std::invalid_argument("Initializer list row count mismatch");
        }

        size_type row = 0;
        for (const auto& row_init : init) {
            if (row_init.size() != Cols) {
                throw std::invalid_argument("Initializer list column count mismatch");
            }

            size_type col = 0;
            for (const auto& value : row_init) {
                data_[row * Cols + col] = value;
                ++col;
            }
            ++row;
        }
    }

    template<typename... Args>
    requires (sizeof...(Args) == size && (std::convertible_to<Args, T> && ...))
    constexpr Matrix(Args... args) noexcept : data_{static_cast<T>(args)...} {}

    // ============================================================================
    // Element Access Operations
    // ============================================================================

    /**
     * @brief Mutable element access using (row, col) notation
     *
     * @param row The row index (0-based)
     * @param col The column index (0-based)
     * @return Reference to the element at position (row, col)
     *
     * Provides mathematical-style element access: matrix(i, j)
     * Uses row-major storage: element (i,j) is at index [i * cols + j]
     *
     * @note No bounds checking in release builds for performance
     * @note constexpr for compile-time access when possible
     * @note noexcept - bounds checking should be done by caller
     */
    constexpr reference operator()(size_type row, size_type col) noexcept {
        return data_[row * Cols + col];
    }

    /**
     * @brief Immutable element access using (row, col) notation
     *
     * @param row The row index (0-based)
     * @param col The column index (0-based)
     * @return Const reference to the element at position (row, col)
     *
     * Const version of element access for read-only operations.
     * Same indexing scheme as the mutable version.
     *
     * @note No bounds checking for performance reasons
     * @note constexpr allows compile-time evaluation
     */
    constexpr const_reference operator()(size_type row, size_type col) const noexcept {
        return data_[row * Cols + col];
    }

    /**
     * @brief Bounds-checked mutable element access
     *
     * @param row The row index (0-based)
     * @param col The column index (0-based)
     * @return Reference to the element at position (row, col)
     * @throws std::out_of_range if row >= rows or col >= cols
     *
     * Safer alternative to operator() that performs bounds checking.
     * Use this when you need guaranteed bounds checking or when
     * working with potentially invalid indices.
     *
     * @note Slightly slower than operator() due to bounds checking
     */
    /**
     * @brief Bounds-checked immutable element access
     *
     * @param row The row index (0-based)
     * @param col The column index (0-based)
     * @return Const reference to the element at position (row, col)
     * @throws std::out_of_range if row >= rows or col >= cols
     *
     * Const version of bounds-checked element access.
     * Provides safety at the cost of a small performance overhead.
     */
    constexpr reference at(size_type row, size_type col) {
        if (row >= Rows || col >= Cols) {
            throw std::out_of_range("Matrix indices out of bounds");
        }
        return data_[row * Cols + col];
    }

    constexpr const_reference at(size_type row, size_type col) const {
        if (row >= Rows || col >= Cols) {
            throw std::out_of_range("Matrix indices out of bounds");
        }
        return data_[row * Cols + col];
    }

    // Raw data access
    constexpr T* data() noexcept { return data_.data(); }
    constexpr const T* data() const noexcept { return data_.data(); }

    // Iterators
    constexpr iterator begin() noexcept { return data_.begin(); }
    constexpr const_iterator begin() const noexcept { return data_.begin(); }
    constexpr const_iterator cbegin() const noexcept { return data_.cbegin(); }
    constexpr iterator end() noexcept { return data_.end(); }
    constexpr const_iterator end() const noexcept { return data_.end(); }
    constexpr const_iterator cend() const noexcept { return data_.cend(); }

    // Row and column access
    constexpr std::span<T, Cols> row(size_type row_idx) noexcept {
        return std::span<T, Cols>{&data_[row_idx * Cols], Cols};
    }

    constexpr std::span<const T, Cols> row(size_type row_idx) const noexcept {
        return std::span<const T, Cols>{&data_[row_idx * Cols], Cols};
    }

    // Matrix operations
    constexpr Matrix& operator+=(const Matrix& other) noexcept {
#ifdef HAS_PARALLEL_STL
        if constexpr (size > 64) {
            std::transform(std::execution::par_unseq,
                         begin(), end(), other.begin(), begin(),
                         std::plus<T>{});
        } else {
#endif
            std::transform(begin(), end(), other.begin(), begin(), std::plus<T>{});
#ifdef HAS_PARALLEL_STL
        }
#endif
        return *this;
    }

    constexpr Matrix& operator-=(const Matrix& other) noexcept {
#ifdef HAS_PARALLEL_STL
        if constexpr (size > 64) {
            std::transform(std::execution::par_unseq,
                         begin(), end(), other.begin(), begin(),
                         std::minus<T>{});
        } else {
#endif
            std::transform(begin(), end(), other.begin(), begin(), std::minus<T>{});
#ifdef HAS_PARALLEL_STL
        }
#endif
        return *this;
    }

    constexpr Matrix& operator*=(T scalar) noexcept {
#ifdef HAS_PARALLEL_STL
        if constexpr (size > 64) {
            std::for_each(std::execution::par_unseq, begin(), end(),
                         [scalar](T& value) { value *= scalar; });
        } else {
#endif
            std::for_each(begin(), end(), [scalar](T& value) { value *= scalar; });
#ifdef HAS_PARALLEL_STL
        }
#endif
        return *this;
    }

    constexpr Matrix& operator/=(T scalar) {
        if (scalar == T{0}) {
            throw std::invalid_argument("Division by zero");
        }
        return *this *= (T{1} / scalar);
    }

    // Binary operators
    constexpr Matrix operator+(const Matrix& other) const noexcept {
        Matrix result = *this;
        result += other;
        return result;
    }

    constexpr Matrix operator-(const Matrix& other) const noexcept {
        Matrix result = *this;
        result -= other;
        return result;
    }

    constexpr Matrix operator*(T scalar) const noexcept {
        Matrix result = *this;
        result *= scalar;
        return result;
    }

    constexpr Matrix operator/(T scalar) const {
        Matrix result = *this;
        result /= scalar;
        return result;
    }

    // Matrix multiplication
    template<size_type OtherCols>
    constexpr Matrix<T, Rows, OtherCols> operator*(const Matrix<T, Cols, OtherCols>& other) const noexcept {
        Matrix<T, Rows, OtherCols> result{};

        for (size_type i = 0; i < Rows; ++i) {
            for (size_type j = 0; j < OtherCols; ++j) {
                T sum = T{0};
                for (size_type k = 0; k < Cols; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }

        return result;
    }

    // Comparison operators
    constexpr bool operator==(const Matrix& other) const noexcept {
        return std::equal(begin(), end(), other.begin());
    }

    constexpr bool operator!=(const Matrix& other) const noexcept {
        return !(*this == other);
    }

    // Matrix properties
    constexpr T trace() const noexcept requires(Rows == Cols) {
        T sum = T{0};
        for (size_type i = 0; i < Rows; ++i) {
            sum += (*this)(i, i);
        }
        return sum;
    }

    constexpr T determinant() const noexcept requires(Rows == Cols && Rows <= 3) {
        if constexpr (Rows == 1) {
            return (*this)(0, 0);
        } else if constexpr (Rows == 2) {
            return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
        } else if constexpr (Rows == 3) {
            return (*this)(0, 0) * ((*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1))
                 - (*this)(0, 1) * ((*this)(1, 0) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 0))
                 + (*this)(0, 2) * ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0));
        }
    }

    constexpr Matrix<T, Cols, Rows> transpose() const noexcept {
        Matrix<T, Cols, Rows> result{};
        for (size_type i = 0; i < Rows; ++i) {
            for (size_type j = 0; j < Cols; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    // Matrix inverse (for small matrices)
    constexpr Matrix inverse() const requires(Rows == Cols && Rows <= 3) {
        const T det = determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon()) {
            throw std::runtime_error("Matrix is not invertible (determinant is zero)");
        }

        if constexpr (Rows == 1) {
            return Matrix{T{1} / (*this)(0, 0)};
        } else if constexpr (Rows == 2) {
            return Matrix{
                { (*this)(1, 1) / det, -(*this)(0, 1) / det},
                {-(*this)(1, 0) / det,  (*this)(0, 0) / det}
            };
        } else if constexpr (Rows == 3) {
            Matrix result{};
            result(0, 0) = ((*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1)) / det;
            result(0, 1) = ((*this)(0, 2) * (*this)(2, 1) - (*this)(0, 1) * (*this)(2, 2)) / det;
            result(0, 2) = ((*this)(0, 1) * (*this)(1, 2) - (*this)(0, 2) * (*this)(1, 1)) / det;
            result(1, 0) = ((*this)(1, 2) * (*this)(2, 0) - (*this)(1, 0) * (*this)(2, 2)) / det;
            result(1, 1) = ((*this)(0, 0) * (*this)(2, 2) - (*this)(0, 2) * (*this)(2, 0)) / det;
            result(1, 2) = ((*this)(0, 2) * (*this)(1, 0) - (*this)(0, 0) * (*this)(1, 2)) / det;
            result(2, 0) = ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0)) / det;
            result(2, 1) = ((*this)(0, 1) * (*this)(2, 0) - (*this)(0, 0) * (*this)(2, 1)) / det;
            result(2, 2) = ((*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0)) / det;
            return result;
        }
    }

    // Zero and identity matrices
    static constexpr Matrix zeros() noexcept {
        return Matrix{T{0}};
    }

    static constexpr Matrix ones() noexcept {
        return Matrix{T{1}};
    }

    static constexpr Matrix identity() noexcept requires(Rows == Cols) {
        Matrix result{};
        for (size_type i = 0; i < Rows; ++i) {
            result(i, i) = T{1};
        }
        return result;
    }

    // Norms
    constexpr T frobenius_norm() const noexcept {
        T sum = T{0};
#ifdef HAS_PARALLEL_STL
        if constexpr (size > 64) {
            sum = std::transform_reduce(std::execution::par_unseq,
                                      begin(), end(), T{0}, std::plus<T>{},
                                      [](T value) { return value * value; });
        } else {
#endif
            sum = std::transform_reduce(begin(), end(), T{0}, std::plus<T>{},
                                      [](T value) { return value * value; });
#ifdef HAS_PARALLEL_STL
        }
#endif
        return std::sqrt(sum);
    }

    // String representation
    std::string to_string() const {
        std::string result = "[\n";
        for (size_type i = 0; i < Rows; ++i) {
            result += "  [";
            for (size_type j = 0; j < Cols; ++j) {
                result += std::format("{:8.4f}", (*this)(i, j));
                if (j < Cols - 1) result += ", ";
            }
            result += "]";
            if (i < Rows - 1) result += ",";
            result += "\n";
        }
        result += "]";
        return result;
    }

private:
    alignas(32) std::array<T, Rows * Cols> data_; // Aligned for SIMD
};

// Type aliases for common matrix sizes
template<typename T>
using Matrix2x2 = Matrix<T, 2, 2>;

template<typename T>
using Matrix3x3 = Matrix<T, 3, 3>;

template<typename T>
using Matrix4x4 = Matrix<T, 4, 4>;

template<typename T>
using Vector2 = Matrix<T, 2, 1>;

template<typename T>
using Vector3 = Matrix<T, 3, 1>;

template<typename T>
using Vector4 = Matrix<T, 4, 1>;

// Common instantiations
using Matrix2x2d = Matrix2x2<double>;
using Matrix3x3d = Matrix3x3<double>;
using Matrix4x4d = Matrix4x4<double>;
using Vector2d = Vector2<double>;
using Vector3d = Vector3<double>;
using Vector4d = Vector4<double>;

using Matrix2x2f = Matrix2x2<float>;
using Matrix3x3f = Matrix3x3<float>;
using Matrix4x4f = Matrix4x4<float>;
using Vector2f = Vector2<float>;
using Vector3f = Vector3<float>;
using Vector4f = Vector4<float>;

// Scalar multiplication (scalar * matrix)
template<std::floating_point T, std::size_t Rows, std::size_t Cols>
constexpr Matrix<T, Rows, Cols> operator*(T scalar, const Matrix<T, Rows, Cols>& matrix) noexcept {
    return matrix * scalar;
}

// Stream output
template<std::floating_point T, std::size_t Rows, std::size_t Cols>
std::ostream& operator<<(std::ostream& os, const Matrix<T, Rows, Cols>& matrix) {
    os << matrix.to_string();
    return os;
}

// Forward declarations for utility functions
template<std::floating_point T, std::size_t N>
Matrix<T, N, N> cholesky_decomposition(const Matrix<T, N, N>& matrix);

template<std::floating_point T, std::size_t N>
Matrix<T, N, 1> forward_substitution(const Matrix<T, N, N>& L, const Matrix<T, N, 1>& b);

template<std::floating_point T, std::size_t N>
Matrix<T, N, 1> backward_substitution(const Matrix<T, N, N>& U, const Matrix<T, N, 1>& b);

template<std::floating_point T, std::size_t N>
bool is_positive_definite(const Matrix<T, N, N>& matrix);

template<std::floating_point T, std::size_t N>
Matrix<T, N, N> matrix_sqrt(const Matrix<T, N, N>& matrix);

template<std::floating_point T>
T condition_number_2x2(const Matrix<T, 2, 2>& matrix);

template<std::floating_point T, std::size_t N>
Matrix<T, N, N> regularize_matrix(const Matrix<T, N, N>& matrix, T regularization = T{1e-10});

template<std::floating_point T, std::size_t N>
Matrix<T, N, N> ensure_symmetric(const Matrix<T, N, N>& matrix);

} // namespace kf::math
