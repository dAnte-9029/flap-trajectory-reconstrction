// Diagnostics of saved GPTR curves; no GP mathematics or solver is implemented.
#include "query_adapter.hpp"
#include <factor/GPIMUFactor.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <functional>
#include <iomanip>
using State=GPState<double>;
using Rows=std::vector<std::vector<double>>;
Rows read(const std::string& path) {
  std::ifstream f(path);if(!f)throw std::runtime_error(path);
  std::string line;std::getline(f,line);Rows rows;
  while(std::getline(f,line)) {std::stringstream s(line);std::string word;std::vector<double> row;
    while(std::getline(s,word,','))row.push_back(std::stod(word));rows.push_back(row);}
  return rows;
}
Vec3 v3(const std::vector<double>& r,int i){return Vec3(r[i],r[i+1],r[i+2]);}
State state(const std::vector<double>& r) {
  return State(r[0],SO3d(Eigen::Quaterniond(r[13],r[10],r[11],r[12])),v3(r,14),v3(r,17),v3(r,1),v3(r,4),v3(r,7));
}
std::vector<double*> params(State& a,State& b,Vec3& bg,Vec3& ba,Vec3& g) {
  return {a.R.data(),a.O.data(),a.S.data(),a.P.data(),a.V.data(),a.A.data(),
          b.R.data(),b.O.data(),b.S.data(),b.P.data(),b.V.data(),b.A.data(),bg.data(),ba.data(),g.data()};
}
using Query=std::function<State(double)>;
// Forced body-derivative replay of the actual MainV2 scheme. No neural model.
struct ReplayState {Vec3 p,v,w;Eigen::Vector4d q;};
ReplayState fromState(const State& x){return {x.P,x.V,x.O,Eigen::Map<const Eigen::Vector4d>(x.R.data())};}
SO3d rotation(const ReplayState& d){Eigen::Quaterniond q(d.q[3],d.q[0],d.q[1],d.q[2]);q.normalize();return SO3d(q);}
ReplayState derivative(const ReplayState& d,const State& truth) {
  SO3d r=rotation(d);Vec3 an=r*(truth.R.inverse()*truth.A);
  Eigen::Quaterniond q(r.unit_quaternion()),omega(0,d.w[0],d.w[1],d.w[2]);
  Eigen::Quaterniond dq=q*omega;return {d.v,an,truth.S,.5*dq.coeffs()};
}
ReplayState addScaled(const ReplayState&a,const ReplayState&b,double h){return {a.p+h*b.p,a.v+h*b.v,a.w+h*b.w,a.q+h*b.q};}
void step(ReplayState& d,const Query& query,double t,double h,int method) {
  State x=query(t);
  if(method==0) {
    auto r=rotation(d);Vec3 an=r*(x.R.inverse()*x.A),wn=d.w+x.S*h;
    d.p+=d.v*h+.5*an*h*h;d.v+=an*h;
    SO3d next=r*SO3d::exp(.5*(d.w+wn)*h);d.q=Eigen::Map<const Eigen::Vector4d>(next.data());d.w=wn;
  } else if(method==1) {
    State mid=query(t+.5*h),end=query(t+h);
    auto k1=derivative(d,x),k2=derivative(addScaled(d,k1,h*.5),mid);
    auto k3=derivative(addScaled(d,k2,h*.5),mid),k4=derivative(addScaled(d,k3,h),end);
    d=addScaled(addScaled(addScaled(addScaled(d,k1,h/6),k2,h/3),k3,h/3),k4,h/6);d.q.normalize();
  } else {
    State end=query(t+h);SO3d r=rotation(d);
    Vec3 deltaV=r*(x.R.inverse()*(end.V-x.V));
    if(method==2) { // Average acceleration gives exact endpoint velocity, not position moment.
      d.p+=d.v*h+.5*deltaV*h;
      Vec3 nextw=d.w+end.O-x.O;
      SO3d next=r*SO3d::exp(.5*(d.w+nextw)*h);d.q=Eigen::Map<const Eigen::Vector4d>(next.data());d.w=nextw;
    } else { // Separate translational moment and rotation increment: endpoint identity only.
      d.p+=d.v*h+r*(x.R.inverse()*(end.P-x.P-x.V*h));
      SO3d next=r*(x.R.inverse()*end.R);d.q=Eigen::Map<const Eigen::Vector4d>(next.data());d.w+=end.O-x.O;
    }
    d.v+=deltaV;
  }
}
Eigen::Vector4d difference(const ReplayState& a,const State& b){Eigen::Vector4d e;e<<(a.p-b.P).norm(),(a.v-b.V).norm(),(rotation(a).inverse()*b.R).log().norm(),(a.w-b.O).norm();return e;}
void integrate(const Query& query,double start,double end,const std::string& file,double offset=.1) {
  std::ofstream out(file);out<<std::setprecision(17)<<"method,horizon,step,n,max_p,max_v,max_R,max_w,rms_p,rms_v,rms_R,rms_w\n";
  const char* names[]={"body_left","body_rk4","interval_average","endpoint_moments"};
  for(int method=0;method<4;++method)for(double horizon:{.02,.1,.2,.5,2.})for(double h:{.02,.01,.005,.0025,.00125}) {
    if(method>=2&&h!=.02)continue;
    Eigen::Vector4d max=Eigen::Vector4d::Zero(),sumsq=max;int n=0;
    for(double t=start+offset;t+horizon<end-.1;t+=.37) {
      ReplayState d=fromState(query(t));int count=std::lround(horizon/h);double delta=horizon/count;
      for(int k=0;k<count;++k)step(d,query,t+k*delta,delta,method);
      auto e=difference(d,query(t+horizon));max=max.cwiseMax(e);sumsq+=e.cwiseProduct(e);++n;
    }
    out<<names[method]<<','<<horizon<<','<<h<<','<<n;for(double v:max)out<<','<<v;for(double v:(sumsq/n).cwiseSqrt())out<<','<<v;out<<'\n';
  }
}
State analytic(double t,bool periodic) {
  if(!periodic)return State(t,SO3d(),Vec3::Zero(),Vec3::Zero(),Vec3(.5*t*t,t*t,-.25*t*t),Vec3(t,2*t,-.5*t),Vec3(1,2,-.5));
  double k=8*M_PI;Vec3 amp(3,2,10),axis=Vec3(0,1,0);
  return State(t,SO3d::exp(axis*(.5/k*std::sin(k*t))),axis*(.5*std::cos(k*t)),axis*(-.5*k*std::sin(k*t)),
    -amp/(k*k)*std::sin(k*t),-amp/k*std::cos(k*t),amp*std::sin(k*t));
}
int main(int argc,char**argv){try {
  if(argc==3&&std::string(argv[1])=="--analytic") {
    if(std::filesystem::exists(argv[2]))throw std::runtime_error("existing output");std::filesystem::create_directories(argv[2]);
    integrate([](double t){return analytic(t,false);},0,4,std::string(argv[2])+"/constant.csv");
    integrate([](double t){return analytic(t,true);},0,4,std::string(argv[2])+"/periodic4hz.csv");return 0;
  }
  if(argc!=6){std::cerr<<"run_dir observations_dir output_dir dt mode(audit|integrate)\n";return 2;}
  std::string run=argv[1],obsdir=argv[2],outdir=argv[3],mode=argv[5];double dt=std::stod(argv[4]);
  if(std::filesystem::exists(outdir))throw std::runtime_error("existing output");std::filesystem::create_directories(outdir);
  auto rows=read(run+"/knots.csv"),dense=read(run+"/trajectory.csv"),sol=read(run+"/solver.csv");
  std::vector<State> knots;for(auto&r:rows)knots.push_back(state(r));
  GaussianProcess gp(dt,Mat3::Identity()*1e4,Mat3::Identity()*1e5);gp.setStartTime(knots.front().t);gp.extendKnotsTo(knots.back().t,State(),false);
  for(size_t i=0;i<knots.size();++i)gp.setKnot(i,knots[i]);
  Query query=[&](double t){return auditedQuery(gp,t);};double start=dense.front()[0],end=dense.back()[0];
  if(mode=="integrate"||mode=="integrate_offgrid") {integrate(query,start,end,outdir+"/integrators.csv",mode=="integrate"?.1:.1037);return 0;}
  if(mode!="audit")throw std::runtime_error("unknown mode");
  Vec3 ba=v3(sol[0],6),bg=v3(sol[0],9),g(0,0,-9.80665);
  std::ofstream out(outdir+"/factor_query.csv");out<<std::setprecision(17)<<"sensor,t,holdout,saved_minus_query,query_minus_factor,residual_jac_mode_difference,pub_minus_sample_ms\n";
  for(int kind=0;kind<2;++kind) {
    std::string name=kind==0?"accel":"gyro";auto pred=read(run+"/"+name+"_predictions.csv"),obs=read(obsdir+"/"+name+".csv");
    for(auto&r:pred) {
      double t=r[0];auto x=query(t);Vec3 o=v3(r,4),prediction=kind==0?(x.R.inverse()*(x.A+g)+ba).eval():(x.O+bg).eval();
      int i=std::floor((t-knots.front().t)/dt);auto p=params(knots[i],knots[i+1],bg,ba,g);
      GPIMUFactor factor(kind==0?o:Vec3::Zero(),kind==0?Vec3::Zero():o,Vec3::Zero(),Vec3::Zero(),kind==0?0:20,kind==0?1/1.5:0,0,0,gp.getGPMixerPtr(),(t-knots[i].t)/dt);
      Eigen::Matrix<double,12,1> res,jres;factor.Evaluate(p.data(),res.data(),nullptr);
      std::vector<std::vector<double>> j;for(int size:factor.parameter_block_sizes())j.push_back(std::vector<double>(12*size));std::vector<double*> ptr;for(auto&a:j)ptr.push_back(a.data());factor.Evaluate(p.data(),jres.data(),ptr.data());
      Vec3 fromfactor=o+(kind==0?res.head<3>().eval()*1.5:res.segment<3>(3).eval()*.05);
      auto row=std::lower_bound(obs.begin(),obs.end(),t,[](const auto&r,double v){return r[0]<v;});if(row==obs.end()||(*row)[0]!=t)throw std::runtime_error("native join mismatch");
      out<<name<<','<<t<<','<<r[1]<<','<<(v3(r,7)-prediction).norm()<<','<<(prediction-fromfactor).norm()<<','<<(res-jres).norm()<<','<<1000*((*row)[6]-t)<<'\n';
    }
  }
  return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
