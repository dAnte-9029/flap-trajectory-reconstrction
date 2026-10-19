#pragma once
#include "pvt_factor.hpp"
#include <ceres/jet.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>

namespace phasejoint {
using Rows=std::vector<std::vector<double>>;
using RowMat=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
inline Rows read(const std::string& path){std::ifstream f(path);if(!f)throw std::runtime_error(path);std::string line;std::getline(f,line);Rows rows;
 while(std::getline(f,line)){std::stringstream ss(line);std::string v;std::vector<double> r;while(std::getline(ss,v,','))r.push_back(std::stod(v));rows.push_back(r);}return rows;}
inline Vec3 vec(const std::vector<double>&r,int i){return Vec3(r[i],r[i+1],r[i+2]);}
inline std::vector<double*> params(State&a,State&b){return {a.R.data(),a.O.data(),a.S.data(),a.P.data(),a.V.data(),a.A.data(),b.R.data(),b.O.data(),b.S.data(),b.P.data(),b.V.data(),b.A.data()};}
inline const std::string stateHeader="t,px,py,pz,vx,vy,vz,ax,ay,az,qx,qy,qz,qw,wx,wy,wz,alphax,alphay,alphaz\n";
inline void writeState(std::ostream&o,const State&x){o<<std::setprecision(17)<<x.t;for(auto v:{x.P,x.V,x.A})for(int j=0;j<3;++j)o<<','<<v[j];for(int j=0;j<4;++j)o<<','<<x.R.data()[j];for(auto v:{x.O,x.S})for(int j=0;j<3;++j)o<<','<<v[j];o<<'\n';}
struct Phase {double p,w,a;};
class PhaseCurve {
 Rows pieces;
 public:
 explicit PhaseCurve(const std::string&p):pieces(read(p)){if(pieces.empty())throw std::runtime_error("Empty phase curve");}
 double lo()const{return pieces.front()[0];} double hi()const{return pieces.back()[1];}
 bool valid(double t)const{return t>=lo()&&t<=hi();}
 Phase query(double t)const{if(!valid(t))throw std::runtime_error("Phase extrapolation prohibited");
  auto it=std::upper_bound(pieces.begin(),pieces.end(),t,[](double v,const auto&r){return v<r[0];});if(it!=pieces.begin())--it;
  const auto&r=*it;double x=t-r[0];return {r[2]+x*(r[3]+x*(r[4]+x*r[5])),r[3]+x*(2*r[4]+3*x*r[5]),2*r[4]+6*x*r[5]};}
};

template<typename T>using V3=Eigen::Matrix<T,3,1>;
template<typename T>using M3=Eigen::Matrix<T,3,3>;
template<typename T>void rightJacobian(const V3<T>&e,const V3<T>&ed,M3<T>&J,M3<T>&Jd){
 using std::sin;using std::cos;using std::sqrt;
 T s=e.squaredNorm(),sd=T(2)*e.dot(ed),aa,bb,ad,bd;
 if(s<T(1e-8)){
  aa=T(.5)-s/T(24)+s*s/T(720)-s*s*s/T(40320);
  bb=T(1./6)-s/T(120)+s*s/T(5040)-s*s*s/T(362880);
  ad=(T(-1./24)+s/T(360)-s*s/T(13440))*sd;
  bd=(T(-1./120)+s/T(2520)-s*s/T(120960))*sd;
 }else{T z=sqrt(s),zd=sd/(T(2)*z);aa=(T(1)-cos(z))/s;bb=(z-sin(z))/(z*z*z);
  ad=(z*sin(z)-T(2)*(T(1)-cos(z)))/(z*z*z)*zd;
  bd=(z*(T(1)-cos(z))-T(3)*(z-sin(z)))/(z*z*z*z)*zd;}
 M3<T>K=Sophus::SO3<T>::hat(e),Kd=Sophus::SO3<T>::hat(ed);
 J=M3<T>::Identity()-aa*K+bb*K*K;Jd=-ad*K-aa*Kd+bd*K*K+bb*(Kd*K+K*Kd);
}

// Fixed coefficients: first 6H entries NED displacement [sin xyz,cos xyz],
// next 6H entries right-tangent FRD rotation. No sensor-specific oscillation.
template<typename T>GPState<T> compose(const GPState<T>&slow,const Phase&ph,const T*c,int H){
 GPState<T>x=slow;V3<T>dp=V3<T>::Zero(),dv=dp,da=dp,e=dp,ed=dp,edd=dp;
 for(int h=1;h<=H;++h){double s=std::sin(h*ph.p),co=std::cos(h*ph.p),w=h*ph.w;
  double basis[2]={s,co},first[2]={w*co,-w*s},second[2]={h*ph.a*co-w*w*s,-h*ph.a*s-w*w*co};
  for(int b=0;b<2;++b)for(int j=0;j<3;++j){int k=6*(h-1)+3*b+j;
   dp[j]+=T(basis[b])*c[k];dv[j]+=T(first[b])*c[k];da[j]+=T(second[b])*c[k];
   e[j]+=T(basis[b])*c[6*H+k];ed[j]+=T(first[b])*c[6*H+k];edd[j]+=T(second[b])*c[6*H+k];}}
 M3<T>J,Jd;rightJacobian(e,ed,J,Jd);auto Q=Sophus::SO3<T>::exp(e);V3<T>wq=J*ed,aq=Jd*ed+J*edd,ws=Q.inverse()*slow.O;
 x.P+=dp;x.V+=dv;x.A+=da;x.R=slow.R*Q;x.O=ws+wq;x.S=Q.inverse()*slow.S-wq.cross(ws)+aq;return x;
}

template<typename T>Eigen::Matrix<T,Eigen::Dynamic,1> predicted(const GPState<T>&x,int kind,const Vec3&lever){
 Eigen::Matrix<T,Eigen::Dynamic,1> y(kind==2?6:3);
 if(kind==0)y=x.R.inverse()*(x.A+Vec3(0,0,-9.80665).cast<T>());
 if(kind==1)y=x.O;
 if(kind==2){y.template head<3>()=x.P+x.R*lever.cast<T>();y.template tail<3>()=x.V+x.R*(x.O.cross(lever.cast<T>()));}return y;
}

class PhaseFactor final:public ceres::CostFunction{
 GPMixerPtr m;double s;Phase ph;int H,kind;Eigen::VectorXd obs;Eigen::MatrixXd W;Vec3 lever;
 public:
 PhaseFactor(GPMixerPtr mixer,double s,Phase ph,int H,int kind,Eigen::VectorXd obs,Eigen::MatrixXd W,Vec3 lever=Vec3::Zero())
 :m(mixer),s(s),ph(ph),H(H),kind(kind),obs(obs),W(W),lever(lever){
  set_num_residuals(kind==2?6:3);for(int k=0;k<12;++k)mutable_parameter_block_sizes()->push_back(k%6==0?4:3);
  mutable_parameter_block_sizes()->insert(mutable_parameter_block_sizes()->end(),{12*H,3,3});}
 bool Evaluate(const double*const*p,double*r,double**jac)const override{
  State a(0),b(m->getDt()),x(s*m->getDt());m->MapParamToState(p,0,a);m->MapParamToState(p,6,b);Blocks ja,jb;
  m->ComputeXtAndJacobiansSO3xR3(a,b,x,ja,jb,jac!=nullptr);
  State total=compose(x,ph,p[12],H);Eigen::VectorXd y=predicted(total,kind,lever);
  if(kind==0)y+=Eigen::Map<const Vec3>(p[14]);if(kind==1)y+=Eigen::Map<const Vec3>(p[13]);
  Eigen::Map<Eigen::VectorXd>(r,num_residuals())=W*(y-obs);if(!jac)return true;
  // Differentiate only the new composition/observation map. GP derivatives
  // remain the pinned upstream analytic mixer plus audited ambient lift.
  using Jet=ceres::Jet<double,54>;V3<Jet>d=V3<Jet>::Zero();for(int j=0;j<3;++j)d[j].v[j]=1;
  GPState<Jet> z(x.t,x.R,x.O,x.S,x.P,x.V,x.A);z.R=x.R.cast<Jet>()*Sophus::SO3<Jet>::exp(d);
  for(int j=0;j<3;++j){z.O[j].v[3+j]=1;z.S[j].v[6+j]=1;z.P[j].v[9+j]=1;z.V[j].v[12+j]=1;z.A[j].v[15+j]=1;}
  std::vector<Jet> c(12*H);for(int k=0;k<12*H;++k){c[k]=Jet(p[12][k]);c[k].v[18+k]=1;}
  auto yp=predicted(compose(z,ph,c.data(),H),kind,lever);Eigen::MatrixXd D(num_residuals(),54);
  for(int j=0;j<num_residuals();++j)D.row(j)=yp[j].v.transpose();D=W*D;
  for(int k=0;k<12;++k)if(jac[k]){int z=k%6;auto&j=k<6?ja:jb;Eigen::MatrixXd local=Eigen::MatrixXd::Zero(num_residuals(),3);
   for(int a=0;a<6;++a)local+=D.block(0,3*a,num_residuals(),3)*j[a][z];
   if(z==0){Eigen::Matrix<double,3,4,Eigen::RowMajor>L;AuditedSO3Manifold().MinusJacobian(p[k],L.data());Eigen::Map<RowMat>(jac[k],num_residuals(),4)=local*L;}
   else Eigen::Map<RowMat>(jac[k],num_residuals(),3)=local;}
  if(jac[12])Eigen::Map<RowMat>(jac[12],num_residuals(),12*H)=D.block(0,18,num_residuals(),12*H);
  for(int k:{13,14})if(jac[k]){Eigen::Map<RowMat>j(jac[k],num_residuals(),3);j.setZero();if((k==13&&kind==1)||(k==14&&kind==0))j=W;}
  return true;
 }
};

class CoefficientPrior:public ceres::CostFunction{
 Eigen::VectorXd precision;
 public:CoefficientPrior(int H,double sp,double sr){set_num_residuals(12*H);mutable_parameter_block_sizes()->push_back(12*H);precision.resize(12*H);
  for(int h=1;h<=H;++h)for(int j=0;j<6;++j){precision[6*(h-1)+j]=h*h/sp;precision[6*H+6*(h-1)+j]=h/sr;}}
 bool Evaluate(const double*const*p,double*r,double**j)const override{Eigen::Map<Eigen::VectorXd>(r,precision.size())=precision.array()*Eigen::Map<const Eigen::VectorXd>(p[0],precision.size()).array();
  if(j&&j[0])Eigen::Map<RowMat>(j[0],precision.size(),precision.size())=precision.asDiagonal();return true;}
};
struct BiasPrior{double sigma;template<typename T>bool operator()(const T*b,T*r)const{for(int i=0;i<3;++i)r[i]=b[i]/T(sigma);return true;}};
inline double jacobianError(ceres::CostFunction&f,std::vector<double*>p,double h,int first=0,int last=100){
 std::vector<RowMat>js;std::vector<double*>jp;for(int n:f.parameter_block_sizes())js.emplace_back(f.num_residuals(),n);for(auto&j:js)jp.push_back(j.data());
 Eigen::VectorXd r(f.num_residuals()),rp(r.size()),rm(r.size());f.Evaluate(p.data(),r.data(),jp.data());double worst=0;
 for(int k=first;k<int(p.size())&&k<last;++k){int n=f.parameter_block_sizes()[k];Eigen::VectorXd saved=Eigen::Map<Eigen::VectorXd>(p[k],n);Eigen::MatrixXd J=js[k];
  if(n==4){Eigen::Matrix<double,4,3,Eigen::RowMajor>P;AuditedSO3Manifold().PlusJacobian(p[k],P.data());J=J*P;}
  for(int c=0;c<(n==4?3:n);++c){for(int sign:{-1,1}){Eigen::Map<Eigen::VectorXd>(p[k],n)=saved;if(n==4){Eigen::Map<SO3d>q(p[k]);Vec3 v=Vec3::Zero();v[c]=sign*h;q=q*SO3d::exp(v);}else p[k][c]+=sign*h;f.Evaluate(p.data(),sign>0?rp.data():rm.data(),nullptr);}
   auto num=((rp-rm)/(2*h)).eval();worst=std::max(worst,((num-J.col(c)).cwiseAbs().array()/(1+num.cwiseAbs().array())).maxCoeff());}Eigen::Map<Eigen::VectorXd>(p[k],n)=saved;}return worst;
}
} // namespace phasejoint
