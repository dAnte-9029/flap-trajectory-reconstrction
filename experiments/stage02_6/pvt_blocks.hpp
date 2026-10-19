#pragma once
#include "phase_motion.hpp"

namespace gnssablation {
using namespace phasejoint;
// A lower block-Cholesky whitening: marginal p, then v conditional on p.
// No explicit inverse, covariance clipping, jitter, or precision submatrix.
struct PVTWhitening {
 Mat3 pp, conditional, K, Wp, Wv;
 Mat6 W;
 explicit PVTWhitening(const Mat6& cov) {
  if(!cov.allFinite() || (cov-cov.transpose()).norm()>1e-12*std::max(1.,cov.norm()))throw std::runtime_error("Nonfinite/asymmetric PVT covariance");
  Eigen::LLT<Mat6> full(cov);
  if(full.info()!=Eigen::Success)throw std::runtime_error("PVT covariance not SPD");
  pp=cov.topLeftCorner<3,3>();Eigen::LLT<Mat3> lp(pp);
  if(lp.info()!=Eigen::Success)throw std::runtime_error("Position covariance not SPD");
  K=lp.solve(cov.topRightCorner<3,3>()).transpose();
  conditional=cov.bottomRightCorner<3,3>()-K*cov.topRightCorner<3,3>();
  conditional=(.5*(conditional+conditional.transpose())).eval();
  Eigen::LLT<Mat3> lv(conditional);
  if(lv.info()!=Eigen::Success)throw std::runtime_error("Conditional velocity covariance not numerically SPD");
  Wp=lp.matrixL().solve(Mat3::Identity());Wv=lv.matrixL().solve(Mat3::Identity());
  W.setZero();W.topLeftCorner<3,3>()=Wp;W.bottomLeftCorner<3,3>()=-Wv*K;W.bottomRightCorner<3,3>()=Wv;
  if(!W.allFinite() || (W*cov*W.transpose()-Mat6::Identity()).norm()>1e-6)throw std::runtime_error("Numerically unresolved covariance whitening");
 }
};
// Reuse the existing complete physical PVT prediction and its analytic state
// Jacobians with identity covariance; only apply a constant whitening map.
class PVTBlock final:public ceres::CostFunction {
 PVTFactor physical;
 Eigen::MatrixXd transform;
 public:
 enum Kind{Position=0,ConditionalVelocity=1,Combined=2};
 PVTBlock(GPMixerPtr m,double s,Vec3 p,Vec3 v,const Mat6&cov,Vec3 lever,Kind kind)
 :physical(m,s,p,kind==Position?Vec3::Zero():v,Mat6::Identity(),lever) {
  PVTWhitening blocks(cov);
  if(kind==Position)transform=blocks.W.topRows<3>();
  else if(kind==ConditionalVelocity)transform=blocks.W.bottomRows<3>();
  else transform=blocks.W;
  set_num_residuals(transform.rows());*mutable_parameter_block_sizes()=physical.parameter_block_sizes();
 }
 const Eigen::MatrixXd& whitening()const{return transform;}
 bool Evaluate(const double*const*p,double*r,double**j)const override {
  Vec6 raw;std::vector<RowMat> scratch(12);std::vector<double*>ptr(12,nullptr);
  if(j)for(int k=0;k<12;++k)if(j[k]){scratch[k].resize(6,parameter_block_sizes()[k]);ptr[k]=scratch[k].data();}
  if(!physical.Evaluate(p,raw.data(),j?ptr.data():nullptr))return false;
  Eigen::Map<Eigen::VectorXd>(r,num_residuals())=transform*raw;
  if(j)for(int k=0;k<12;++k)if(j[k])Eigen::Map<RowMat>(j[k],num_residuals(),parameter_block_sizes()[k])=transform*scratch[k];
  return true;
 }
};
inline PVTBlock* positionFactor(GPMixerPtr m,double s,Vec3 p,const Mat6&cov,Vec3 lever){return new PVTBlock(m,s,p,Vec3::Zero(),cov,lever,PVTBlock::Position);}
}
