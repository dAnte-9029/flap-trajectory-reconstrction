// Synthetic tests against the external pinned implementation, not a GP rewrite.
#include <GaussianProcess.hpp>
#include <factor/GPIMUFactor.h>
#include <factor/GPMotionPriorTwoKnotsFactor.h>
#include "query_adapter.hpp"
#include <iomanip>
#include <stdexcept>

using State = GPState<double>;
using Blocks = std::vector<std::vector<Mat3>>;
using RowMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
static int failures = 0;
static void check(const std::string& name, double error, double tolerance) {
  bool pass = std::isfinite(error) && error <= tolerance;
  std::cout << name << ',' << std::setprecision(17) << error << ',' << tolerance
            << ',' << (pass ? "PASS" : "FAIL") << '\n';
  failures += !pass;
}
static void issue(const std::string& name, double error) {
  std::cout << name << ',' << std::setprecision(17) << error << ",,KNOWN_ISSUE\n";
  if (!std::isfinite(error) || error < 1e-8) ++failures;
}
static Vec3& vectorPart(State& x, int j) {
  switch(j) {case 1:return x.O; case 2:return x.S; case 3:return x.P;
    case 4:return x.V; case 5:return x.A; default:throw std::runtime_error("rotation has no vector reference");}
}
static Eigen::Matrix<double,18,1> difference(const State& base, State other) {
  Eigen::Matrix<double,18,1> d;
  d.head<3>() = (base.R.inverse()*other.R).log();
  State copy = base;
  for(int j=1;j<6;++j) d.segment<3>(3*j)=vectorPart(other,j)-vectorPart(copy,j);
  return d;
}
static State perturb(State x, int j, int axis, double epsilon) {
  Vec3 d=Vec3::Zero(); d[axis]=epsilon;
  if(j==0) x.R=x.R*SO3d::exp(d); else vectorPart(x,j)+=d;
  return x;
}
static State interpolate(const GPMixer& m,const State& a,const State& b,double t) {
  State x(t); Blocks ja,jb;
  m.ComputeXtAndJacobiansSO3xR3(a,b,x,ja,jb,false);
  return x;
}
static State polynomial(double t) {
  // Quintic scalar times fixed vectors: analytic p/v/a and commuting R/w/alpha.
  double f=.2+.4*t-.3*t*t+.2*pow(t,3)-.1*pow(t,4)+.03*pow(t,5);
  double df=.4-.6*t+.6*t*t-.4*pow(t,3)+.15*pow(t,4);
  double ddf=-.6+1.2*t-1.2*t*t+.6*pow(t,3);
  Vec3 axis=Vec3(1,2,-1).normalized(), scale(1,-2,.5);
  return State(t,SO3d::exp(axis*f),axis*df,axis*ddf,scale*f,scale*df,scale*ddf);
}
static State periodic(double t) {
  const double k=8*std::acos(-1.0); // 4 Hz: within the audited wing fundamental.
  double a=.08*std::sin(k*t),ad=.08*k*std::cos(k*t),add=-k*k*a;
  double b=.06*std::cos(k*t),bd=-.06*k*std::sin(k*t),bdd=-k*k*b;
  SO3d ry=SO3d::exp(Vec3(0,b,0));
  Vec3 u=ry.inverse()*Vec3(ad,0,0);
  Vec3 omega=u+Vec3(0,bd,0);
  Vec3 alpha=ry.inverse()*Vec3(add,0,0)-Vec3(0,bd,0).cross(u)+Vec3(0,bdd,0);
  Vec3 p(.03*std::sin(k*t),.02*std::cos(k*t),.1*t+.01*std::sin(k*t));
  Vec3 v(.03*k*std::cos(k*t),-.02*k*std::sin(k*t),.1+.01*k*std::cos(k*t));
  Vec3 acc(-.03*k*k*std::sin(k*t),-.02*k*k*std::cos(k*t),-.01*k*k*std::sin(k*t));
  return State(t,SO3d::exp(Vec3(a,0,0))*ry,omega,alpha,p,v,acc);
}
static double interpolationJacobian(const GPMixer& m,State a,State b,double t,double h) {
  State x(t); Blocks ja,jb; m.ComputeXtAndJacobiansSO3xR3(a,b,x,ja,jb);
  double worst=0;
  for(int end=0;end<2;++end) for(int j=0;j<6;++j) for(int axis=0;axis<3;++axis) {
    State plus=interpolate(m,end==0?perturb(a,j,axis,h):a,end==1?perturb(b,j,axis,h):b,t);
    State minus=interpolate(m,end==0?perturb(a,j,axis,-h):a,end==1?perturb(b,j,axis,-h):b,t);
    Eigen::Matrix<double,18,1> numeric=(difference(x,plus)-difference(x,minus))/(2*h);
    for(int out=0;out<6;++out) for(int c=0;c<3;++c) {
      double n=numeric[3*out+c], analytic=(end?jb:ja)[out][j](c,axis);
      worst=std::max(worst,std::abs(analytic-n)/(1+std::abs(n)));
    }
  }
  return worst;
}
static std::vector<double*> parameters(State& a,State& b,Vec3& bg,Vec3& ba,Vec3& g) {
  return {a.R.data(),a.O.data(),a.S.data(),a.P.data(),a.V.data(),a.A.data(),
          b.R.data(),b.O.data(),b.S.data(),b.P.data(),b.V.data(),b.A.data(),bg.data(),ba.data(),g.data()};
}
static double factorJacobian(ceres::CostFunction& factor,std::vector<double*> p,double h,bool ambient=false) {
  const int rows=factor.num_residuals(), blocks=factor.parameter_block_sizes().size();
  std::vector<RowMat> matrices; matrices.reserve(blocks);
  std::vector<double*> jac;
  for(int j=0;j<blocks;++j) matrices.emplace_back(rows,factor.parameter_block_sizes()[j]);
  for(auto& m:matrices)jac.push_back(m.data());
  Eigen::VectorXd residual(rows),rp(rows),rm(rows);
  factor.Evaluate(p.data(),residual.data(),jac.data());
  double worst=0;
  for(int j=0;j<blocks;++j) {
    int size=factor.parameter_block_sizes()[j];
    Eigen::VectorXd saved=Eigen::Map<Eigen::VectorXd>(p[j],size);
    for(int c=0;c<3;++c) {
      for(int sign : {-1,1}) {
        Eigen::Map<Eigen::VectorXd>(p[j],size)=saved;
        if(size==4) {
          SO3d q=Eigen::Map<const SO3d>(saved.data()); Vec3 d=Vec3::Zero();d[c]=sign*h;
          Eigen::Map<SO3d> mapped(p[j]); mapped=q*SO3d::exp(d);
        } else p[j][c]+=sign*h;
        factor.Evaluate(p.data(),sign==1?rp.data():rm.data(),nullptr);
      }
      Eigen::Map<Eigen::VectorXd>(p[j],size)=saved;
      Eigen::VectorXd numeric=(rp-rm)/(2*h);
      Eigen::VectorXd analytic=matrices[j].col(c);
      if(size==4 && ambient) {
        Eigen::Matrix<double,4,3,Eigen::RowMajor> plus;
        AuditedSO3Manifold().PlusJacobian(p[j],plus.data());
        analytic=matrices[j]*plus.col(c);
      }
      // Upstream: identity-pad manifold. Adapter: actual quaternion derivative.
      for(int r=0;r<rows;++r) worst=std::max(worst,std::abs(numeric[r]-analytic[r])/(1+std::abs(numeric[r])));
    }
  }
  // Exercise null Jacobian requests, including either quaternion separately.
  for(int keep=0;keep<blocks;++keep) {
    std::vector<double*> partial(blocks,nullptr);partial[keep]=jac[keep];
    factor.Evaluate(p.data(),residual.data(),partial.data());
  }
  return worst;
}
static void manifoldProbe() {
  GPSO3dLocalParameterization manifold;
  SO3d q=SO3d::exp(Vec3(.3,-.2,.1));
  Eigen::Matrix<double,4,3,Eigen::RowMajor> claimed,numeric;
  manifold.PlusJacobian(q.data(),claimed.data());
  for(int c=0;c<3;++c) {
    Vec3 d=Vec3::Zero();d[c]=1e-7; SO3d plus,minus;
    manifold.Plus(q.data(),d.data(),plus.data());d=-d;
    manifold.Plus(q.data(),d.data(),minus.data());
    numeric.col(c)=(plus.unit_quaternion().coeffs()-minus.unit_quaternion().coeffs())/2e-7;
  }
  issue("upstream_manifold_ambient_derivative_mismatch",(claimed-numeric).cwiseAbs().maxCoeff());
  AuditedSO3Manifold compatible;
  compatible.PlusJacobian(q.data(),claimed.data());
  check("adapter_manifold_plus_jacobian",(claimed-numeric).cwiseAbs().maxCoeff(),1e-8);
  Eigen::Matrix<double,3,4,Eigen::RowMajor> minus,minus_numeric;
  compatible.MinusJacobian(q.data(),minus.data());
  // Off-manifold perturbations projected onto unit quaternions define the
  // standard smooth extension used for this ambient numerical check.
  for(int c=0;c<4;++c) {
    Eigen::Vector4d qp=q.unit_quaternion().coeffs(),qm=qp;qp[c]+=1e-7;qm[c]-=1e-7;
    qp.normalize();qm.normalize();Vec3 dp,dm;
    compatible.Minus(qp.data(),q.data(),dp.data());compatible.Minus(qm.data(),q.data(),dm.data());
    minus_numeric.col(c)=(dp-dm)/2e-7;
  }
  check("adapter_manifold_minus_jacobian",(minus-minus_numeric).cwiseAbs().maxCoeff(),1e-8);
  check("adapter_manifold_inverse",(minus*claimed-Mat3::Identity()).norm(),1e-12);
}
static void publicQueryProbe() {
  GaussianProcess gp(.25,Mat3::Identity(),Mat3::Identity());
  State a=polynomial(.3),b=polynomial(.55);
  gp.setStartTime(a.t);gp.setKnot(0,a);gp.extendOneKnot(b);
  State direct=interpolate(*gp.getGPMixerPtr(),a,b,.43), public_x=gp.getStateAt(.43);
  issue("upstream_getStateAt_wrong_endpoint_alpha",(direct.S-public_x.S).norm());
  // Distinguish near-knot snapping from the wrong endpoint state.
  double t=a.t+gp.getDt()*.0005;
  State snapped=gp.getStateAt(t), truth=polynomial(t);
  issue("upstream_getStateAt_near_knot_snapping",(snapped.P-truth.P).norm());
  double err=0;
  for(double ts : {a.t,.43,b.t}) err=std::max(err,difference(polynomial(ts),auditedQuery(gp,ts)).norm());
  check("adapter_query_away_from_mixer_cutoff",err,1e-7);
  // The query wrapper alone does NOT fix the independent 1 ms cutoff.
#ifdef STAGE01_FIX_PSI_CUTOFF
  check("patched_mixer_submillisecond_acceleration_error",
        (auditedQuery(gp,t).A-polynomial(t).A).norm(),1e-8);
  State before=auditedQuery(gp,a.t+.001-1e-9),after=auditedQuery(gp,a.t+.001+1e-9);
  check("patched_mixer_1ms_acceleration_continuity",(before.A-after.A).norm(),1e-8);
#else
  issue("upstream_mixer_submillisecond_acceleration_error",
        (auditedQuery(gp,t).A-polynomial(t).A).norm());
  State before=auditedQuery(gp,a.t+.001-1e-9),after=auditedQuery(gp,a.t+.001+1e-9);
  issue("upstream_mixer_1ms_acceleration_jump",(before.A-after.A).norm());
#endif
  int rejected=0;
  for(double ts : {a.t-1e-6,b.t+1e-6,std::numeric_limits<double>::quiet_NaN()})
    try {auditedQuery(gp,ts);} catch(const std::out_of_range&) {++rejected;}
  check("adapter_no_extrapolation",3-rejected,0);
}
static void factorProbes() {
  auto m=std::make_shared<GPMixer>(.05,Mat3::Identity(),Mat3::Identity());
  State a=periodic(.13),b=periodic(.18);a.t=0;b.t=.05;
  Vec3 bg(.01,-.02,.03),ba(.1,-.2,.05),g(0,0,-9.80665);
  auto p=parameters(a,b,bg,ba,g);
  State x=interpolate(*m,a,b,.019);
  Vec3 measured=x.R.inverse()*(x.A+g)+ba,gyro=x.O+bg;
  GPIMUFactor imu(measured,gyro,ba,bg,2,3,4,5,m,.019/.05);
  Eigen::Matrix<double,12,1> r;
  imu.Evaluate(p.data(),r.data(),nullptr);check("imu_truth_and_bias",r.norm(),1e-10);
  check("imu_composed_local_jacobian",factorJacobian(imu,p,1e-6),5e-4);
  AmbientFactorAdapter adapted(std::make_unique<GPIMUFactor>(measured,gyro,ba,bg,2,3,4,5,m,.019/.05));
  check("imu_ambient_adapter_composed_jacobian",factorJacobian(adapted,p,1e-6,true),5e-4);
  GPMotionPriorTwoKnotsFactor prior(m);
  check("motion_prior_composed_local_jacobian",factorJacobian(prior,p,1e-6),5e-4);
  // Separate measurement times using the original factor with inactive weights.
  for(int kind=0;kind<2;++kind) {
    double t=kind==0?.013:.01505;
    State at=interpolate(*m,a,b,t);
    GPIMUFactor separated(at.R.inverse()*(at.A+g)+ba,at.O+bg,ba,bg,
                         kind==1?2:0,kind==0?3:0,0,0,m,t/.05);
    separated.Evaluate(p.data(),r.data(),nullptr);
    check(kind==0?"async_accel_original_factor":"async_gyro_original_factor",r.norm(),1e-10);
    check(kind==0?"async_accel_jacobian":"async_gyro_jacobian",factorJacobian(separated,p,1e-6),5e-4);
  }
  State zero;State end=zero;end.t=.05; Vec3 z=Vec3::Zero();
  auto pz=parameters(zero,end,z,z,g);
  GPIMUFactor stationary(g,z,z,z,1,1,0,0,m,.37);
  stationary.Evaluate(pz.data(),r.data(),nullptr);check("ned_stationary_specific_force",r.norm(),1e-12);
  // A small real Ceres solve: fixed analytic trajectory, estimate only two biases.
  Vec3 estimate_g=Vec3::Zero(),estimate_a=Vec3::Zero();
  auto ps=parameters(a,b,estimate_g,estimate_a,g);
  ceres::Problem problem;
  for(double s : {.13,.37,.83}) {
    State truth=interpolate(*m,a,b,s*.05);
    problem.AddResidualBlock(new GPIMUFactor(truth.R.inverse()*(truth.A+g)+ba,
       truth.O+bg,Vec3::Zero(),Vec3::Zero(),1,1,0,0,m,s),nullptr,ps);
  }
  for(int j=0;j<12;++j) problem.SetParameterBlockConstant(ps[j]);
  problem.SetParameterBlockConstant(g.data());
  ceres::Solver::Options options; options.linear_solver_type=ceres::DENSE_QR;
  options.max_num_iterations=10;options.num_threads=1;
  ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);
  check("ceres_bias_only_solution",(estimate_g-bg).norm()+(estimate_a-ba).norm(),1e-7);
  check("ceres_solution_usable",summary.IsSolutionUsable()?0:1,0);
}

// Synthetic-only anchors exercise ordinary AutoDiff alongside upstream analytic
// factors. These are not proposed independent measurements from real PX4 states.
struct SyntheticStateAnchor {
  State truth;
  template<typename T>
  bool operator()(const T* q,const T* w,const T* alpha,const T* p,
                  const T* v,const T* a,T* residuals) const {
    Eigen::Map<Sophus::SO3<T> const> rotation(q);
    Eigen::Map<Eigen::Matrix<T,18,1>> r(residuals);
    r.template head<3>()=(truth.R.template cast<T>().inverse()*rotation).log();
    const T* blocks[]={w,alpha,p,v,a};
    const Vec3* reference[]={&truth.O,&truth.S,&truth.P,&truth.V,&truth.A};
    for(int j=0;j<5;++j)
      r.template segment<3>(3*(j+1))=Eigen::Map<const Eigen::Matrix<T,3,1>>(blocks[j])-reference[j]->template cast<T>();
    return true;
  }
};

static void jointSyntheticSolve() {
  double dt=.1;
  Vec3 axis=Vec3(1,-2,1).normalized(), acceleration(.3,-.2,.1), velocity(.4,.1,-.2);
  State truth_a(0,SO3d::exp(axis*.2),axis*.4,axis*.3,Vec3(1,2,-1),velocity,acceleration);
  State truth_b(dt,truth_a.R*SO3d::exp(axis*(.4*dt+.15*dt*dt)),axis*(.4+.3*dt),axis*.3,
                truth_a.P+velocity*dt+.5*acceleration*dt*dt,velocity+acceleration*dt,acceleration);
  State a=truth_a,b=truth_b;
  for(int j=0;j<6;++j) {a=perturb(a,j,j%3,.015);b=perturb(b,j,(j+1)%3,-.02);}
  Vec3 bg=Vec3::Zero(),ba=Vec3::Zero(),g(0,0,-9.80665);
  auto p=parameters(a,b,bg,ba,g);
  auto mixer=std::make_shared<GPMixer>(dt,Mat3::Identity(),Mat3::Identity());
  ceres::Problem problem;
  using Anchor=ceres::AutoDiffCostFunction<SyntheticStateAnchor,18,4,3,3,3,3,3>;
  problem.AddResidualBlock(new Anchor(new SyntheticStateAnchor{truth_a}),nullptr,
                           std::vector<double*>(p.begin(),p.begin()+6));
  problem.AddResidualBlock(new Anchor(new SyntheticStateAnchor{truth_b}),nullptr,
                           std::vector<double*>(p.begin()+6,p.begin()+12));
  problem.AddResidualBlock(new AmbientFactorAdapter(std::make_unique<GPMotionPriorTwoKnotsFactor>(mixer)),nullptr,
                           std::vector<double*>(p.begin(),p.begin()+12));
  for(double s : {.17,.43,.81}) {
    State x=interpolate(*mixer,truth_a,truth_b,s*dt);
    problem.AddResidualBlock(new AmbientFactorAdapter(std::make_unique<GPIMUFactor>(
       x.R.inverse()*(x.A+g),x.O,ba,bg,1,1,0,0,mixer,s)),nullptr,p);
  }
  problem.SetManifold(a.R.data(),new AuditedSO3Manifold());
  problem.SetManifold(b.R.data(),new AuditedSO3Manifold());
  for(int j=12;j<15;++j)problem.SetParameterBlockConstant(p[j]);
  ceres::Solver::Options options;options.linear_solver_type=ceres::DENSE_QR;
  options.max_num_iterations=40;options.num_threads=1;
  options.function_tolerance=1e-12;options.gradient_tolerance=1e-12;options.parameter_tolerance=1e-12;
  ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);
  check("ceres_joint_state_solution",std::max(difference(truth_a,a).norm(),difference(truth_b,b).norm()),1e-6);
  check("ceres_joint_state_usable",summary.IsSolutionUsable()?0:1,0);
}

static void boundaryAndIntegrationChecks() {
  const double dt=.05,t0=.13,tc=t0+dt,h=1e-9;
  State left=periodic(t0),middle=periodic(tc),right=periodic(tc+dt);
  GPMixer mixer(dt,Mat3::Identity(),Mat3::Identity());
  State l=interpolate(mixer,left,middle,tc-h),r=interpolate(mixer,middle,right,tc+h);
  const char* names[]={"R","omega","alpha","p","v","a"};
  auto jump=difference(l,r);
  for(int j=0;j<6;++j)check(std::string("knot_boundary_")+names[j],jump.segment<3>(3*j).norm(),1e-4);
  // Numerical integration is a separate check from interpolation/derivatives.
  // Start beyond the original PSI cutoff, and converge the quadrature step.
  const double start=t0+.002,end=t0+dt-.002;
  State initial=interpolate(mixer,left,middle,start),final=interpolate(mixer,left,middle,end);
  double previous=1e9;
  for(int n : {50,100,200}) {
    const double step=(end-start)/n;
    Vec3 dp=Vec3::Zero(),dv=Vec3::Zero(),dw=Vec3::Zero();SO3d orientation=initial.R;
    for(int k=0;k<n;++k) {
      State x=interpolate(mixer,left,middle,start+(k+.5)*step);
      dp+=x.V*step;dv+=x.A*step;dw+=x.S*step;
      orientation=orientation*SO3d::exp(x.O*step);
    }
    double error=std::max({(dp-(final.P-initial.P)).norm(),(dv-(final.V-initial.V)).norm(),
       (dw-(final.O-initial.O)).norm(),(orientation.inverse()*final.R).log().norm()});
    std::cout<<"midpoint_integration_n"<<n<<','<<error<<",,MEASURED_ERROR\n";
    if(n>50)check("midpoint_convergence_n"+std::to_string(n),error/previous,.3);
    previous=error;
  }
  check("midpoint_integration_final",previous,1e-5);
}

static void smallAngleApproximationCheck() {
  GPMixer m(.05,Mat3::Identity(),Mat3::Identity());
  Vec3 axis=Vec3(1,-2,.5).normalized(),u=axis*.0005,v(.7,-.2,.4);
  Mat3 numeric;
  for(int c=0;c<3;++c) {
    Vec3 plus=u,minus=u;plus[c]+=1e-7;minus[c]-=1e-7;
    numeric.col(c)=(m.Jr(plus)*v-m.Jr(minus)*v)/2e-7;
  }
  double mismatch=(m.DJrUV_DU(u,v)-numeric).cwiseAbs().maxCoeff();
  Vec3 ul=axis*(.001-1e-10),ur=axis*(.001+1e-10);
  double branch_jump=(m.DJrUV_DU(ul,v)-m.DJrUV_DU(ur,v)).cwiseAbs().maxCoeff();
  std::cout<<"upstream_small_angle_H_derivative_error,"<<mismatch<<",,APPROXIMATION_LIMIT\n";
  std::cout<<"upstream_small_angle_H_branch_jump,"<<branch_jump<<",,APPROXIMATION_LIMIT\n";
  if(!std::isfinite(mismatch) || !std::isfinite(branch_jump))++failures;
}
int main() {
  std::cout << "test,value,tolerance,status\n";
  State a=polynomial(.3),b=polynomial(.55);GPMixer m(.25,Mat3::Identity(),Mat3::Identity());
  Eigen::Matrix<double,6,1> errors=Eigen::Matrix<double,6,1>::Zero();
  for(int i=0;i<=20;++i) {
    double t=.3+.25*i/20.; auto d=difference(polynomial(t),interpolate(m,a,b,t));
    for(int j=0;j<6;++j) errors[j]=std::max(errors[j],d.segment<3>(3*j).norm());
  }
  const char* names[]={"R","omega","alpha","p","v","a"};
  for(int j=0;j<6;++j)check(std::string("quintic_")+names[j],errors[j],1e-7);
  for(double h : {1e-5,1e-6})check("polynomial_jacobian_h"+std::to_string(h),interpolationJacobian(m,a,b,.43,h),5e-4);
  for(double angle : {0.,.0005,.002,.1}) {
    State x,y; x.t=0;y.t=.05;y.R=SO3d::exp(Vec3(angle,0,0));
    x.O=Vec3(.2,-.3,.1);y.O=Vec3(-.1,.2,.3);x.S=Vec3(.4,.2,-.3);y.S=Vec3(.1,-.3,.2);
    GPMixer small(.05,Mat3::Identity(),Mat3::Identity());
    check("small_rotation_jacobian_"+std::to_string(angle),interpolationJacobian(small,x,y,.019,1e-6),5e-4);
  }
  for(double dt : {.1,.05,.02}) {
    GPMixer mx(dt,Mat3::Identity(),Mat3::Identity());
    Eigen::Matrix<double,6,1> err=Eigen::Matrix<double,6,1>::Zero();
    double kinematic_error=0,jacobian_error=0;
    for(int k=0;k<5;++k) {
      double t0=.13+k*dt;State x=periodic(t0),y=periodic(t0+dt);
      for(double s : {.1,.37,.8}) {
        double t=t0+s*dt,h=1e-6;State at=interpolate(mx,x,y,t);
        auto e=difference(periodic(t),at);
        for(int j=0;j<6;++j)err[j]=std::max(err[j],e.segment<3>(3*j).norm());
        State plus=interpolate(mx,x,y,t+h),minus=interpolate(mx,x,y,t-h);
        kinematic_error=std::max({kinematic_error,((plus.P-minus.P)/(2*h)-at.V).norm(),
          ((plus.V-minus.V)/(2*h)-at.A).norm(),
          ((minus.R.inverse()*plus.R).log()/(2*h)-at.O).norm(),
          ((plus.O-minus.O)/(2*h)-at.S).norm()});
      }
      jacobian_error=std::max(jacobian_error,interpolationJacobian(mx,x,y,t0+.37*dt,1e-6));
    }
    for(int j=0;j<6;++j)std::cout<<"periodic_dt"<<dt<<'_'<<names[j]<<','<<err[j]<<",,MEASURED_ERROR\n";
    check("periodic_kinematics_dt"+std::to_string(dt),kinematic_error,2e-4);
    check("periodic_jacobian_dt"+std::to_string(dt),jacobian_error,5e-4);
  }
  manifoldProbe();publicQueryProbe();factorProbes();jointSyntheticSolve();boundaryAndIntegrationChecks();smallAngleApproximationCheck();
  return failures ? 1 : 0;
}
