#pragma once
#include "query_adapter.hpp"
using State = GPState<double>;
using Mat6 = Eigen::Matrix<double,6,6>;
using Vec6 = Eigen::Matrix<double,6,1>;
using Blocks = std::vector<std::vector<Mat3>>;
// Thin observation factor: no new GP formula, no GTSAM/Ceres mixing.
// Antenna lever is FROM main IMU reference TO antenna, in body FRD.
class PVTFactor final : public ceres::CostFunction {
 public:
  PVTFactor(GPMixerPtr mixer,double s,Vec3 position,Vec3 velocity,Mat6 covariance,Vec3 lever)
      : m(mixer),s(s),p(position),v(velocity),lever(lever) {
    Eigen::LLT<Mat6> llt(covariance);
    if(llt.info()!=Eigen::Success)throw std::runtime_error("PVT covariance not SPD");
    W=llt.matrixL().solve(Mat6::Identity());
    set_num_residuals(6);
    for(int k=0;k<12;++k)mutable_parameter_block_sizes()->push_back(k%6==0?4:3);
  }
  bool Evaluate(const double*const* pars,double* residual,double** jac)const override {
    State a(0),b(m->getDt()),x(s*m->getDt());m->MapParamToState(pars,0,a);m->MapParamToState(pars,6,b);
    Blocks ja,jb;m->ComputeXtAndJacobiansSO3xR3(a,b,x,ja,jb,jac!=nullptr);
    Vec6 r;r.head<3>()=x.P+x.R*lever-p;r.tail<3>()=x.V+x.R*(x.O.cross(lever))-v;
    Eigen::Map<Vec6> rr(residual);rr=W*r;
    if(!jac)return true;
    Eigen::Matrix<double,6,3> dr=Eigen::Matrix<double,6,3>::Zero(),dw=dr,dp=dr,dv=dr;
    dr.topRows<3>()=-x.R.matrix()*SO3d::hat(lever);
    dr.bottomRows<3>()=-x.R.matrix()*SO3d::hat(x.O.cross(lever));
    dw.bottomRows<3>()=-x.R.matrix()*SO3d::hat(lever);
    dp.topRows<3>()=Mat3::Identity();dv.bottomRows<3>()=Mat3::Identity();
    for(int k=0;k<12;++k)if(jac[k]) {
      const auto& j=k<6?ja:jb;int z=k%6;
      Eigen::Matrix<double,6,3> local=W*(dr*j[0][z]+dw*j[1][z]+dp*j[3][z]+dv*j[4][z]);
      if(z==0){Eigen::Matrix<double,3,4,Eigen::RowMajor> lift;AuditedSO3Manifold().MinusJacobian(pars[k],lift.data());
        Eigen::Map<Eigen::Matrix<double,6,4,Eigen::RowMajor>> out(jac[k]);out=local*lift;}
      else {Eigen::Map<Eigen::Matrix<double,6,3,Eigen::RowMajor>> out(jac[k]);out=local;}
    }
    return true;
  }
 private:
  GPMixerPtr m;double s;Vec3 p,v,lever;Mat6 W;
};
