#pragma once
// Explicit build-only compatibility boundary for pinned GPTR. utility.h pulls
// ROS/PCL into otherwise standalone numerical headers. The numerical modules
// need only sparse block assignment from that file. Supply it here, and use
// its published guard to skip the unrelated middleware. This is NOT an
// unmodified full upstream build; no GP, Lie derivative, or factor is replaced.
#define _Util_LIDAR_ODOMETRY_H_
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <memory>
#include <vector>
namespace Util {
template<typename T>
void SetSparseMatBlock(Eigen::SparseMatrix<T>& destination, int row, int col,
                      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& values,
                      bool compress = true) {
  for (Eigen::Index c = 0; c < values.cols(); ++c)
    for (Eigen::Index r = 0; r < values.rows(); ++r)
      destination.coeffRef(row + r, col + c) = values(r, c);
  if (compress) destination.makeCompressed();
}
}
