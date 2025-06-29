#include "Matrix.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace kf::math {

// Explicit template instantiations for common types
template class Matrix<double, 2, 2>;
template class Matrix<double, 3, 3>;
template class Matrix<double, 4, 4>;
template class Matrix<double, 2, 1>;
template class Matrix<double, 3, 1>;
template class Matrix<double, 4, 1>;

template class Matrix<float, 2, 2>;
template class Matrix<float, 3, 3>;
template class Matrix<float, 4, 4>;
template class Matrix<float, 2, 1>;
template class Matrix<float, 3, 1>;
template class Matrix<float, 4, 1>;

// Additional utility functions for matrices

/**
 * @brief Cholesky decomposition for positive definite matrices
 * @tparam T Floating point type
 * @tparam N Matrix dimension
 * @param matrix Input positive definite matrix
 * @return Lower triangular matrix L such that A = L * L^T
 */
template<std::floating_point T, std::size_t N>
Matrix<T, N, N> cholesky_decomposition(const Matrix<T, N, N>& matrix) {
    Matrix<T, N, N> L{};

    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j <= i; ++j) {
            if (i == j) {
                // Diagonal elements
                T sum = T{0};
                for (std::size_t k = 0; k < j; ++k) {
                    sum += L(j, k) * L(j, k);
                }
                T diagonal_value = matrix(j, j) - sum;
                if (diagonal_value <= T{0}) {
                    throw std::runtime_error("Matrix is not positive definite");
                }
                L(j, j) = std::sqrt(diagonal_value);
            } else {
                // Lower triangular elements
                T sum = T{0};
                for (std::size_t k = 0; k < j; ++k) {
                    sum += L(i, k) * L(j, k);
                }
                L(i, j) = (matrix(i, j) - sum) / L(j, j);
            }
        }
    }

    return L;
}

/**
 * @brief Solve linear system Ax = b using forward substitution
 * @tparam T Floating point type
 * @tparam N Matrix dimension
 * @param L Lower triangular matrix
 * @param b Right-hand side vector
 * @return Solution vector x
 */
template<std::floating_point T, std::size_t N>
Matrix<T, N, 1> forward_substitution(const Matrix<T, N, N>& L, const Matrix<T, N, 1>& b) {
    Matrix<T, N, 1> x{};

    for (std::size_t i = 0; i < N; ++i) {
        T sum = T{0};
        for (std::size_t j = 0; j < i; ++j) {
            sum += L(i, j) * x(j, 0);
        }
        x(i, 0) = (b(i, 0) - sum) / L(i, i);
    }

    return x;
}

/**
 * @brief Solve linear system Ax = b using backward substitution
 * @tparam T Floating point type
 * @tparam N Matrix dimension
 * @param U Upper triangular matrix
 * @param b Right-hand side vector
 * @return Solution vector x
 */
template<std::floating_point T, std::size_t N>
Matrix<T, N, 1> backward_substitution(const Matrix<T, N, N>& U, const Matrix<T, N, 1>& b) {
    Matrix<T, N, 1> x{};

    for (std::ptrdiff_t i = N - 1; i >= 0; --i) {
        T sum = T{0};
        for (std::size_t j = i + 1; j < N; ++j) {
            sum += U(i, j) * x(j, 0);
        }
        x(i, 0) = (b(i, 0) - sum) / U(i, i);
    }

    return x;
}

/**
 * @brief Check if a matrix is positive definite
 * @tparam T Floating point type
 * @tparam N Matrix dimension
 * @param matrix Input matrix
 * @return true if positive definite, false otherwise
 */
template<std::floating_point T, std::size_t N>
bool is_positive_definite(const Matrix<T, N, N>& matrix) {
    try {
        cholesky_decomposition(matrix);
        return true;
    } catch (const std::runtime_error&) {
        return false;
    }
}

/**
 * @brief Compute matrix square root using Cholesky decomposition
 * @tparam T Floating point type
 * @tparam N Matrix dimension
 * @param matrix Positive definite matrix
 * @return Matrix square root
 */
template<std::floating_point T, std::size_t N>
Matrix<T, N, N> matrix_sqrt(const Matrix<T, N, N>& matrix) {
    return cholesky_decomposition(matrix);
}

/**
 * @brief Compute condition number (ratio of largest to smallest singular value)
 * This is a simplified version for 2x2 matrices commonly used in Kalman filtering
 * @tparam T Floating point type
 * @param matrix Input 2x2 matrix
 * @return Condition number
 */
template<std::floating_point T>
T condition_number_2x2(const Matrix<T, 2, 2>& matrix) {
    // For 2x2 matrix, compute eigenvalues to estimate condition number
    T a = matrix(0, 0);
    T b = matrix(0, 1);
    T c = matrix(1, 0);
    T d = matrix(1, 1);

    T trace = a + d;
    T det = a * d - b * c;

    if (std::abs(det) < std::numeric_limits<T>::epsilon()) {
        return std::numeric_limits<T>::infinity();
    }

    T discriminant = trace * trace - 4 * det;
    if (discriminant < 0) {
        // Complex eigenvalues, use trace and determinant
        return std::abs(trace) / std::abs(det);
    }

    T sqrt_disc = std::sqrt(discriminant);
    T lambda1 = (trace + sqrt_disc) / 2;
    T lambda2 = (trace - sqrt_disc) / 2;

    T max_eigenval = std::max(std::abs(lambda1), std::abs(lambda2));
    T min_eigenval = std::min(std::abs(lambda1), std::abs(lambda2));

    if (min_eigenval < std::numeric_limits<T>::epsilon()) {
        return std::numeric_limits<T>::infinity();
    }

    return max_eigenval / min_eigenval;
}

/**
 * @brief Regularize a matrix by adding small values to diagonal
 * @tparam T Floating point type
 * @tparam N Matrix dimension
 * @param matrix Input matrix
 * @param regularization Regularization parameter
 * @return Regularized matrix
 */
template<std::floating_point T, std::size_t N>
Matrix<T, N, N> regularize_matrix(const Matrix<T, N, N>& matrix, T regularization) {
    Matrix<T, N, N> result = matrix;
    for (std::size_t i = 0; i < N; ++i) {
        result(i, i) += regularization;
    }
    return result;
}

/**
 * @brief Ensure matrix is symmetric by averaging with its transpose
 * @tparam T Floating point type
 * @tparam N Matrix dimension
 * @param matrix Input matrix
 * @return Symmetrized matrix
 */
template<std::floating_point T, std::size_t N>
Matrix<T, N, N> ensure_symmetric(const Matrix<T, N, N>& matrix) {
    Matrix<T, N, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            result(i, j) = (matrix(i, j) + matrix(j, i)) / T{2};
        }
    }
    return result;
}

// Explicit instantiations for utility functions
template Matrix<double, 2, 2> cholesky_decomposition<double, 2>(const Matrix<double, 2, 2>&);
template Matrix<double, 3, 3> cholesky_decomposition<double, 3>(const Matrix<double, 3, 3>&);
template Matrix<double, 2, 1> forward_substitution<double, 2>(const Matrix<double, 2, 2>&, const Matrix<double, 2, 1>&);
template Matrix<double, 3, 1> forward_substitution<double, 3>(const Matrix<double, 3, 3>&, const Matrix<double, 3, 1>&);
template Matrix<double, 2, 1> backward_substitution<double, 2>(const Matrix<double, 2, 2>&, const Matrix<double, 2, 1>&);
template Matrix<double, 3, 1> backward_substitution<double, 3>(const Matrix<double, 3, 3>&, const Matrix<double, 3, 1>&);
template bool is_positive_definite<double, 2>(const Matrix<double, 2, 2>&);
template bool is_positive_definite<double, 3>(const Matrix<double, 3, 3>&);
template Matrix<double, 2, 2> matrix_sqrt<double, 2>(const Matrix<double, 2, 2>&);
template Matrix<double, 3, 3> matrix_sqrt<double, 3>(const Matrix<double, 3, 3>&);
template double condition_number_2x2<double>(const Matrix<double, 2, 2>&);
template Matrix<double, 2, 2> regularize_matrix<double, 2>(const Matrix<double, 2, 2>&, double);
template Matrix<double, 3, 3> regularize_matrix<double, 3>(const Matrix<double, 3, 3>&, double);
template Matrix<double, 2, 2> ensure_symmetric<double, 2>(const Matrix<double, 2, 2>&);
template Matrix<double, 3, 3> ensure_symmetric<double, 3>(const Matrix<double, 3, 3>&);

// Float versions
template Matrix<float, 2, 2> cholesky_decomposition<float, 2>(const Matrix<float, 2, 2>&);
template Matrix<float, 3, 3> cholesky_decomposition<float, 3>(const Matrix<float, 3, 3>&);
template Matrix<float, 2, 1> forward_substitution<float, 2>(const Matrix<float, 2, 2>&, const Matrix<float, 2, 1>&);
template Matrix<float, 3, 1> forward_substitution<float, 3>(const Matrix<float, 3, 3>&, const Matrix<float, 3, 1>&);
template Matrix<float, 2, 1> backward_substitution<float, 2>(const Matrix<float, 2, 2>&, const Matrix<float, 2, 1>&);
template Matrix<float, 3, 1> backward_substitution<float, 3>(const Matrix<float, 3, 3>&, const Matrix<float, 3, 1>&);
template bool is_positive_definite<float, 2>(const Matrix<float, 2, 2>&);
template bool is_positive_definite<float, 3>(const Matrix<float, 3, 3>&);
template Matrix<float, 2, 2> matrix_sqrt<float, 2>(const Matrix<float, 2, 2>&);
template Matrix<float, 3, 3> matrix_sqrt<float, 3>(const Matrix<float, 3, 3>&);
template float condition_number_2x2<float>(const Matrix<float, 2, 2>&);
template Matrix<float, 2, 2> regularize_matrix<float, 2>(const Matrix<float, 2, 2>&, float);
template Matrix<float, 3, 3> regularize_matrix<float, 3>(const Matrix<float, 3, 3>&, float);
template Matrix<float, 2, 2> ensure_symmetric<float, 2>(const Matrix<float, 2, 2>&);
template Matrix<float, 3, 3> ensure_symmetric<float, 3>(const Matrix<float, 3, 3>&);

} // namespace kf::math
