#ifndef LEGO_LOAM_BOR_DEGENERACY_UTILS_H
#define LEGO_LOAM_BOR_DEGENERACY_UTILS_H

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace lego_loam_bor
{

template<int Dimension>
struct DegeneracyProjectionT
{
  Eigen::Matrix<float, Dimension, Dimension> projection =
    Eigen::Matrix<float, Dimension, Dimension>::Identity();
  Eigen::Matrix<float, Dimension, 1> eigenvalues =
    Eigen::Matrix<float, Dimension, 1>::Zero();
  Eigen::Matrix<float, Dimension, Dimension> eigenvectors =
    Eigen::Matrix<float, Dimension, Dimension>::Identity();
  std::array<bool, Dimension> degenerate_eigendirections{};
  bool is_degenerate = false;
};

using DegeneracyProjection = DegeneracyProjectionT<6>;

template<int Dimension>
inline DegeneracyProjectionT<Dimension> computeDegeneracyProjection(
  const Eigen::Matrix<float, Dimension, Dimension> & normal_matrix,
  const Eigen::Matrix<float, Dimension, 1> & eigenvalue_thresholds)
{
  DegeneracyProjectionT<Dimension> result;
  const Eigen::SelfAdjointEigenSolver<
    Eigen::Matrix<float, Dimension, Dimension>> solver(normal_matrix);
  if (solver.info() != Eigen::Success) {
    result.projection.setZero();
    result.eigenvalues.setConstant(-1.0F);
    result.degenerate_eigendirections.fill(true);
    result.is_degenerate = true;
    return result;
  }

  // Eigen returns eigenvalues in increasing order and eigenvectors in columns.
  result.eigenvalues = solver.eigenvalues();
  result.eigenvectors = solver.eigenvectors();

  Eigen::Matrix<float, Dimension, Dimension> observable_mask =
    Eigen::Matrix<float, Dimension, Dimension>::Identity();
  for (int i = 0; i < Dimension; ++i) {
    if (!std::isfinite(result.eigenvalues(i)) ||
        result.eigenvalues(i) < eigenvalue_thresholds(i)) {
      observable_mask(i, i) = 0.0F;
      result.degenerate_eigendirections[static_cast<std::size_t>(i)] = true;
      result.is_degenerate = true;
    }
  }

  // V stores eigenvectors as columns, so project with V D V^T.
  result.projection = result.eigenvectors * observable_mask * result.eigenvectors.transpose();
  return result;
}

inline int dominantDofForEigenvector(
  const Eigen::Matrix<float, 6, 6> & eigenvectors, const int column)
{
  int dominant_index = 0;
  float dominant_magnitude = std::abs(eigenvectors(0, column));
  for (int row = 1; row < 6; ++row) {
    const float magnitude = std::abs(eigenvectors(row, column));
    if (magnitude > dominant_magnitude) {
      dominant_magnitude = magnitude;
      dominant_index = row;
    }
  }
  return dominant_index;
}

}  // namespace lego_loam_bor

#endif  // LEGO_LOAM_BOR_DEGENERACY_UTILS_H
