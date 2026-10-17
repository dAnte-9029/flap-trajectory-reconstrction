#pragma once
#include <GaussianProcess.hpp>
#include <stdexcept>

// Bypass getStateAt's wrong right-end angular acceleration and near-knot snap.
// This delegates all interpolation mathematics to the pinned upstream mixer.
inline GPState<double> auditedQuery(GaussianProcess& trajectory, double time) {
  if (!std::isfinite(time) || time < trajectory.getMinTime() || time > trajectory.getMaxTime())
    throw std::out_of_range("No trajectory extrapolation is authorized");
  if (time == trajectory.getMaxTime()) return trajectory.getKnot(trajectory.getNumKnots()-1);
  auto interval = trajectory.computeTimeIndex(time);
  auto left = trajectory.getKnot(interval.first);
  auto right = trajectory.getKnot(interval.first+1);
  GPState<double> result(time);
  std::vector<std::vector<Mat3>> jl,jr;
  trajectory.getGPMixerPtr()->ComputeXtAndJacobiansSO3xR3(left,right,result,jl,jr,false);
  return result;
}

// Right-multiplicative SO(3) exponential coordinates in radians, quaternion xyzw.
// A real ambient derivative is required when mixing standard/autodiff factors.
class AuditedSO3Manifold final : public GPSO3dLocalParameterization {
 public:
  bool PlusJacobian(const double* x,double* out) const override {
    Eigen::Map<const Eigen::Vector4d> q(x);
    Eigen::Map<Eigen::Matrix<double,4,3,Eigen::RowMajor>> j(out);
    j.topRows<3>()=.5*(q[3]*Mat3::Identity()+SO3d::hat(q.head<3>()));
    j.bottomRows<1>()=-.5*q.head<3>().transpose();
    return true;
  }
  bool MinusJacobian(const double* x,double* out) const override {
    Eigen::Matrix<double,4,3,Eigen::RowMajor> plus;
    PlusJacobian(x,plus.data());
    Eigen::Map<Eigen::Matrix<double,3,4,Eigen::RowMajor>> minus(out);
    minus=4*plus.transpose();
    return true;
  }
};

// Convert upstream's padded LOCAL quaternion Jacobians into ambient Jacobians.
// Only wrap the pinned hand-written factors with that convention, never AutoDiff.
class AmbientFactorAdapter final : public ceres::CostFunction {
 public:
  explicit AmbientFactorAdapter(std::unique_ptr<ceres::CostFunction> inner)
      : inner_(std::move(inner)) {
    set_num_residuals(inner_->num_residuals());
    *mutable_parameter_block_sizes()=inner_->parameter_block_sizes();
  }
  bool Evaluate(const double* const* p,double* r,double** j) const override {
    if (!inner_->Evaluate(p,r,j)) return false;
    if (!j) return true;
    AuditedSO3Manifold manifold;
    for(size_t k=0;k<parameter_block_sizes().size();++k) {
      if (parameter_block_sizes()[k]!=4 || !j[k]) continue;
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,4,Eigen::RowMajor>> block(j[k],num_residuals(),4);
      Eigen::MatrixXd tangent=block.leftCols<3>();
      Eigen::Matrix<double,3,4,Eigen::RowMajor> lift;
      manifold.MinusJacobian(p[k],lift.data());
      block=tangent*lift;
    }
    return true;
  }
 private:
  std::unique_ptr<ceres::CostFunction> inner_;
};
