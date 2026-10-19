#include "phase_motion.hpp"
#include <factor/GPIMUFactor.h>
using namespace phasejoint;
State slowTruth(double t){State x(t);Vec3 u(.31,-.17,.13),v(-.11,.23,.09);auto Q=SO3d::exp(v*t*t);x.R=SO3d::exp(u*t)*Q;x.O=Q.inverse()*u+2*t*v;x.S=-2*t*v.cross(Q.inverse()*u)+2*v;
x.P=Vec3(t+.3*t*t,.2*t*t*t,-.1*t*t);x.V=Vec3(1+.6*t,.6*t*t,-.2*t);x.A=Vec3(.6,1.2*t,-.2);return x;}
Phase ph(double t){return {24*t+.7*t*t+.1*std::sin(2*t),24+1.4*t+.2*std::cos(2*t),1.4-.4*std::sin(2*t)};}
int main(){double worstD=0,worstCoef=0,worstEndpoint=0,worstPVT=0,worstParity=0;Vec3 lever(.21,-.09,.05),ba(.12,-.07,.05),bg(.003,-.002,.001);
 std::cout<<"test,H,step,p,v,R,omega\n";
 for(int H:{1,3}){std::vector<double>c(12*H);for(int i=0;i<12*H;++i)c[i]=(i<6*H?.006:.018)*std::sin(1.7*(i+1));auto query=[&](double t){return compose(slowTruth(t),ph(t),c.data(),H);};
  for(double h:{1e-4,1e-5,1e-6}){Eigen::Vector4d err=Eigen::Vector4d::Zero();for(double t=.01317;t<1.97;t+=.01373){auto a=query(t-h),x=query(t),b=query(t+h);Eigen::Vector4d e;e<<((b.P-a.P)/(2*h)-x.V).norm(),((b.V-a.V)/(2*h)-x.A).norm(),((a.R.inverse()*b.R).log()/(2*h)-x.O).norm(),((b.O-a.O)/(2*h)-x.S).norm();err=err.cwiseMax(e);auto pa=predicted(a,2,lever),pb=predicted(b,2,lever),px=predicted(x,2,lever);if(h==1e-6)worstPVT=std::max(worstPVT,((pb.head<3>()-pa.head<3>())/(2*h)-px.tail<3>()).norm());}std::cout<<"derivative,"<<H<<','<<h;for(double v:err)std::cout<<','<<v;std::cout<<'\n';if(h==1e-6)worstD=std::max(worstD,err.maxCoeff());}
  for(double h:{.002,.0005,.000125}){double t=.037,duration=.4;auto a=query(t),b=query(t+duration);Vec3 dp=Vec3::Zero(),dv=dp,dw=dp;auto R=a.R;int n=std::lround(duration/h);for(int i=0;i<n;++i){auto x=query(t+(i+.5)*h);dp+=h*x.V;dv+=h*x.A;dw+=h*x.S;R=R*SO3d::exp(h*x.O);}std::cout<<"integral,"<<H<<','<<h<<','<<(a.P+dp-b.P).norm()<<','<<(a.V+dv-b.V).norm()<<','<<(R.inverse()*b.R).log().norm()<<','<<(a.O+dw-b.O).norm()<<'\n';}
  auto mixer=std::make_shared<GPMixer>(.05,1e3*Mat3::Identity(),1e4*Mat3::Identity());
  for(double origin:{0.,.173,1.131})for(double s:{.0001,.37,.9999}){auto a=slowTruth(origin),b=slowTruth(origin+.05);auto p=params(a,b);p.insert(p.end(),{c.data(),bg.data(),ba.data()});for(int kind:{0,1,2}){int dim=kind==2?6:3;Eigen::MatrixXd W=Eigen::MatrixXd::Identity(dim,dim);W(0,0)=2;W(1,0)=.2;PhaseFactor f(mixer,s,ph(origin+s*.05),H,kind,Eigen::VectorXd::Zero(dim),W,lever);worstCoef=std::max(worstCoef,jacobianError(f,p,1e-6,12,15));worstEndpoint=std::max(worstEndpoint,jacobianError(f,p,1e-6,0,12));}}
  std::fill(c.begin(),c.end(),0.);auto a=slowTruth(.2),b=slowTruth(.25);auto p=params(a,b);p.insert(p.end(),{c.data(),bg.data(),ba.data()});for(int kind:{0,1,2}){int dim=kind==2?6:3;PhaseFactor f(mixer,.43,ph(.2215),H,kind,Eigen::VectorXd::Zero(dim),Eigen::MatrixXd::Identity(dim,dim),lever);worstCoef=std::max(worstCoef,jacobianError(f,p,1e-6,12,15));}
 }
 // With zero periodic amplitude, the new factors must exactly reproduce
 // original asynchronous IMU and covariance-whitened, lever-arm PVT residuals.
 {int H=3;std::vector<double>c(36,0);auto a=slowTruth(.31),b=slowTruth(.36);auto mixer=std::make_shared<GPMixer>(.05,1e3*Mat3::Identity(),1e4*Mat3::Identity());Vec3 g(0,0,-9.80665),obs(.2,-.3,.4);auto base=params(a,b),p=base;p.insert(p.end(),{c.data(),bg.data(),ba.data()});auto old=base;old.insert(old.end(),{bg.data(),ba.data(),g.data()});
 for(int kind:{0,1}){PhaseFactor f(mixer,.37,ph(.3285),H,kind,obs,Mat3::Identity()*2,lever);GPIMUFactor original(kind==0?obs:Vec3::Zero(),kind==1?obs:Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),kind==1?2:0,kind==0?2:0,0,0,mixer,.37);Vec3 r;Eigen::Matrix<double,12,1>ro;f.Evaluate(p.data(),r.data(),nullptr);original.Evaluate(old.data(),ro.data(),nullptr);worstParity=std::max(worstParity,(r-ro.segment<3>(kind*3)).norm());}
 Mat6 L=Mat6::Identity();L(2,0)=.2;L(4,1)=.1;L(0,0)=.2;Mat6 cov=L*L.transpose(),W=L.inverse();Vec6 ob;ob<<obs,obs;PhaseFactor f(mixer,.37,ph(.3285),H,2,ob,W,lever);PVTFactor original(mixer,.37,obs,obs,cov,lever);Vec6 r,ro;f.Evaluate(p.data(),r.data(),nullptr);original.Evaluate(base.data(),ro.data(),nullptr);worstParity=std::max(worstParity,(r-ro).norm());
 }
 std::cout<<"zero_periodic_original_factor_parity="<<worstParity<<'\n';
 std::cout<<"max_derivative="<<worstD<<" max_pvt_derivative="<<worstPVT<<" coefficient_bias_jac="<<worstCoef<<" endpoint_jac="<<worstEndpoint<<'\n';
 return worstParity<1e-10&&worstD<1e-5&&worstPVT<1e-5&&worstCoef<1e-5&&worstEndpoint<5e-3?0:1;
}
