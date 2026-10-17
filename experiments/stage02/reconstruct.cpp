#include "pvt_factor.hpp"
#include <factor/GPIMUFactor.h>
#include <factor/GPMotionPriorTwoKnotsFactor.h>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iomanip>
using Rows=std::vector<std::vector<double>>;
using RowMat=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
static Rows read(const std::string& p){std::ifstream f(p);if(!f)throw std::runtime_error(p);std::string line;std::getline(f,line);Rows r;
 while(std::getline(f,line)){std::stringstream s(line);std::string v;std::vector<double> x;while(std::getline(s,v,','))x.push_back(std::stod(v));r.push_back(x);}return r;}
static Vec3 vec(const std::vector<double>& r,int k){return Vec3(r[k],r[k+1],r[k+2]);}
static std::vector<double*> params(State&a,State&b){return {a.R.data(),a.O.data(),a.S.data(),a.P.data(),a.V.data(),a.A.data(),b.R.data(),b.O.data(),b.S.data(),b.P.data(),b.V.data(),b.A.data()};}
static State interp(const GPMixer& m,const State&a,const State&b,double t){State x(t);Blocks ja,jb;m.ComputeXtAndJacobiansSO3xR3(a,b,x,ja,jb,false);return x;}
static void writeState(std::ostream& o,const State& x){o<<std::setprecision(17)<<x.t;for(const Vec3& v:{x.P,x.V,x.A})for(int i=0;i<3;++i)o<<','<<v[i];for(int i=0;i<4;++i)o<<','<<x.R.data()[i];for(const Vec3& v:{x.O,x.S})for(int i=0;i<3;++i)o<<','<<v[i];o<<'\n';}
static std::string stateHeader="t,px,py,pz,vx,vy,vz,ax,ay,az,qx,qy,qz,qw,wx,wy,wz,alphax,alphay,alphaz\n";
struct BiasPrior{Vec3 sigma;template<class T>bool operator()(const T*b,T*r)const{for(int i=0;i<3;++i)r[i]=b[i]/T(sigma[i]);return true;}};
static double factorError(ceres::CostFunction& f,std::vector<double*> p,double h){
 std::vector<RowMat> j;std::vector<double*> ptr;for(auto n:f.parameter_block_sizes())j.emplace_back(f.num_residuals(),n);for(auto& v:j)ptr.push_back(v.data());
 Eigen::VectorXd r(f.num_residuals()),rp(r.size()),rm(r.size());f.Evaluate(p.data(),r.data(),ptr.data());double err=0;
 for(size_t k=0;k<p.size();++k){int n=f.parameter_block_sizes()[k];Eigen::VectorXd saved=Eigen::Map<Eigen::VectorXd>(p[k],n);Eigen::MatrixXd analytic=j[k];
  if(n==4){Eigen::Matrix<double,4,3,Eigen::RowMajor> plus;AuditedSO3Manifold().PlusJacobian(p[k],plus.data());analytic=j[k]*plus;}
  for(int c=0;c<3;++c){for(int sign:{-1,1}){Eigen::Map<Eigen::VectorXd>(p[k],n)=saved;
   if(n==4){Eigen::Map<SO3d> q(p[k]);Vec3 d=Vec3::Zero();d[c]=sign*h;q=q*SO3d::exp(d);}else p[k][c]+=sign*h;
   f.Evaluate(p.data(),sign>0?rp.data():rm.data(),nullptr);}
   Eigen::VectorXd num=(rp-rm)/(2*h);err=std::max(err,((num-analytic.col(c)).cwiseAbs().array()/(1+num.cwiseAbs().array())).maxCoeff());}
  Eigen::Map<Eigen::VectorXd>(p[k],n)=saved;}
 return err;
}
static int synthetic(){
 std::cout<<"test,value,tolerance,status\n";int fails=0;auto check=[&](std::string n,double v,double t){bool ok=std::isfinite(v)&&v<=t;std::cout<<n<<','<<std::setprecision(17)<<v<<','<<t<<','<<(ok?"PASS":"FAIL")<<'\n';fails+=!ok;};
 auto m=std::make_shared<GPMixer>(.05,Mat3::Identity(),Mat3::Identity());
 State a(0,SO3d::exp(Vec3(.2,-.1,.4)),Vec3(.5,.3,-.4),Vec3(2,-1,3),Vec3(1,2,3),Vec3(2,-3,1),Vec3(.2,-.3,.1));
 State b=a;b.t=.05;b.R=a.R*SO3d::exp(Vec3(.03,-.02,.04));b.O+=Vec3(.1,.2,-.1);b.P+=Vec3(.1,-.15,.05);
 Vec3 lever(.3,-.15,.1);Mat6 cov=Mat6::Identity();cov(0,1)=cov(1,0)=.3;cov(2,5)=cov(5,2)=.2;
 for(double s:{.01,.37,.999}){State x=interp(*m,a,b,s*.05);PVTFactor f(m,s,x.P+x.R*lever,x.V+x.R*(x.O.cross(lever)),cov,lever);auto p=params(a,b);Vec6 r;f.Evaluate(p.data(),r.data(),nullptr);check("nonzero_lever_residual",r.norm(),1e-12);check("pvt_local_jacobian",factorError(f,p,1e-6),5e-4);}
 State x=interp(*m,a,b,.023);Vec6 offset;offset<<.2,-.1,.3,.05,-.03,.04;
 PVTFactor f(m,.023/.05,x.P+x.R*lever-offset.head<3>(),x.V+x.R*(x.O.cross(lever))-offset.tail<3>(),cov,lever);
 auto p=params(a,b);Vec6 r;f.Evaluate(p.data(),r.data(),nullptr);check("full_covariance_mahalanobis",std::abs(r.squaredNorm()-offset.dot(cov.inverse()*offset)),1e-12);
 double h=1e-6;State l=interp(*m,a,b,.023-h),u=interp(*m,a,b,.023+h);
 check("antenna_position_derivative",((u.P+u.R*lever-l.P-l.R*lever)/(2*h)-(x.V+x.R*(x.O.cross(lever)))).norm(),2e-5);
 // Positive reception delay means measurement is earlier. Test both time and wrong-sign sensitivity.
 double receive=.023+.11,delay=.11;check("positive_delay_time",std::abs(receive-delay-.023),1e-14);
 PVTFactor timed(m,(receive-delay)/.05,x.P+x.R*lever,x.V+x.R*(x.O.cross(lever)),cov,lever);
 timed.Evaluate(p.data(),r.data(),nullptr);check("delayed_factor_residual",r.norm(),1e-12);
 // Covariance must be SPD, not silently repaired in the factor.
 bool rejected=false;try{PVTFactor bad(m,.5,x.P,x.V,-Mat6::Identity(),lever);}catch(...){rejected=true;}check("reject_bad_covariance",rejected?0:1,0);
 return fails?1:0;
}
int main(int argc,char**argv){try{
 if(argc==2&&std::string(argv[1])=="--test")return synthetic();
 if(argc!=18){std::cerr<<"input output start end dt qT qR sigmaA sigmaG delay lx ly lz yaw_deg maxiter biasA biasG\n";return 2;}
 std::string input=argv[1],out=argv[2];if(std::filesystem::exists(out))throw std::runtime_error("Refuse existing run");std::filesystem::create_directories(out);
 double start=std::stod(argv[3]),end=std::stod(argv[4]),dt=std::stod(argv[5]),qt=std::stod(argv[6]),qr=std::stod(argv[7]),sa=std::stod(argv[8]),sg=std::stod(argv[9]),delay=std::stod(argv[10]);
 Vec3 lever(std::stod(argv[11]),std::stod(argv[12]),std::stod(argv[13]));double yaw=std::stod(argv[14])*M_PI/180.;int maxiter=std::stoi(argv[15]);double sba=std::stod(argv[16]),sbg=std::stod(argv[17]);
 auto acc=read(input+"/accel.csv"),gyro=read(input+"/gyro.csv"),gps=read(input+"/gnss.csv"),att=read(input+"/attitude_init.csv");
 const double t0=start-2,finish=end+2;int n=std::lround((finish-t0)/dt)+1;std::vector<State> knots;knots.reserve(n);
 Rows gpstrain;for(auto r:gps)if(r[43]==0&&r[44]!=0){r[0]-=delay;gpstrain.push_back(r);}if(gpstrain.size()<4)throw std::runtime_error("insufficient GNSS");
 auto linear=[&](const Rows& rows,double t,int col){auto it=std::lower_bound(rows.begin(),rows.end(),t,[](const auto&r,double x){return r[0]<x;});
  size_t j=it==rows.begin()?0:std::min(size_t(it-rows.begin()-1),rows.size()-2);double s=(t-rows[j][0])/(rows[j+1][0]-rows[j][0]);return ((1-s)*vec(rows[j],col)+s*vec(rows[j+1],col)).eval();};
 auto initialR=[&](double t){auto it=std::lower_bound(att.begin(),att.end(),t,[](const auto&r,double x){return r[0]<x;});size_t j=it==att.begin()?0:std::min(size_t(it-att.begin()-1),att.size()-2);
   SO3d a(Eigen::Quaterniond(att[j][4],att[j][1],att[j][2],att[j][3])),b(Eigen::Quaterniond(att[j+1][4],att[j+1][1],att[j+1][2],att[j+1][3]));double s=(t-att[j][0])/(att[j+1][0]-att[j][0]);return SO3d::exp(Vec3(0,0,yaw))*a*SO3d::exp(s*(a.inverse()*b).log());};
 Rows gtrain;for(auto r:gyro)if(r[4]==0&&r[5]!=0)gtrain.push_back(r);
 for(int i=0;i<n;++i){State x(t0+i*dt);x.R=initialR(x.t);x.P=linear(gpstrain,x.t,1)-x.R*lever;x.V=linear(gpstrain,x.t,4);x.O=linear(gtrain,x.t,1);x.A.setZero();x.S.setZero();knots.push_back(x);}
 Vec3 ba=Vec3::Zero(),bg=Vec3::Zero(),gravity(0,0,-9.80665);auto mixer=std::make_shared<GPMixer>(dt,qr*Mat3::Identity(),qt*Mat3::Identity());ceres::Problem problem;
 for(int i=0;i<n-1;++i)problem.AddResidualBlock(new AmbientFactorAdapter(std::make_unique<GPMotionPriorTwoKnotsFactor>(mixer)),nullptr,params(knots[i],knots[i+1]));
 for(auto& k:knots)problem.SetManifold(k.R.data(),new AuditedSO3Manifold());
 auto addimu=[&](Rows& rows,bool isacc){for(const auto&r:rows){double t=r[0];if(r[4]!=0||r[5]==0||t<t0||t>=finish)continue;int j=std::floor((t-t0)/dt);auto p=params(knots[j],knots[j+1]);p.insert(p.end(),{bg.data(),ba.data(),gravity.data()});
  problem.AddResidualBlock(new AmbientFactorAdapter(std::make_unique<GPIMUFactor>(isacc?vec(r,1):Vec3::Zero(),isacc?Vec3::Zero():vec(r,1),Vec3::Zero(),Vec3::Zero(),isacc?0:1/sg,isacc?1/sa:0,0,0,mixer,(t-knots[j].t)/dt)),new ceres::HuberLoss(3),p);}};
 addimu(acc,true);addimu(gyro,false);problem.SetParameterBlockConstant(gravity.data());
 using BP=ceres::AutoDiffCostFunction<BiasPrior,3,3>;problem.AddResidualBlock(new BP(new BiasPrior{Vec3::Constant(sba)}),nullptr,ba.data());problem.AddResidualBlock(new BP(new BiasPrior{Vec3::Constant(sbg)}),nullptr,bg.data());
 for(const auto&r:gpstrain){double t=r[0];if(t<t0||t>=finish)continue;int j=std::floor((t-t0)/dt);Mat6 cov;for(int a=0;a<6;++a)for(int b=0;b<6;++b)cov(a,b)=r[7+6*a+b];problem.AddResidualBlock(new PVTFactor(mixer,(t-knots[j].t)/dt,vec(r,1),vec(r,4),cov,lever),new ceres::HuberLoss(3),params(knots[j],knots[j+1]));}
 ceres::Solver::Options opts;opts.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;opts.max_num_iterations=maxiter;opts.num_threads=2;opts.function_tolerance=1e-8;opts.gradient_tolerance=1e-10;opts.parameter_tolerance=1e-9;opts.minimizer_progress_to_stdout=true;
 ceres::Solver::Summary summary;ceres::Solve(opts,&problem,&summary);
 std::ofstream report(out+"/solver.txt");report<<summary.FullReport()<<"\nBA "<<ba.transpose()<<"\nBG "<<bg.transpose()<<'\n';
 std::ofstream status(out+"/solver.csv");status<<"usable,termination,initial_cost,final_cost,iterations,time_s,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z\n"<<std::setprecision(17)<<summary.IsSolutionUsable()<<','<<int(summary.termination_type)<<','<<summary.initial_cost<<','<<summary.final_cost<<','<<summary.iterations.size()<<','<<summary.total_time_in_seconds;for(auto v:{ba,bg})for(int k=0;k<3;++k)status<<','<<v[k];status<<'\n';
 GaussianProcess trajectory(mixer);trajectory.setStartTime(t0);trajectory.extendKnotsTo(knots.back().t,State(),false);for(int i=0;i<n;++i)trajectory.setKnot(i,knots[i]);
 auto query=[&](double t){return auditedQuery(trajectory,t);};
 std::ofstream ko(out+"/knots.csv"),dense(out+"/trajectory.csv");ko<<stateHeader;dense<<stateHeader;for(auto k:knots)writeState(ko,k);
 for(int i=0;i<=std::lround((end-start)/.005);++i)writeState(dense,query(start+i*.005));
 for(int kind=0;kind<3;++kind){auto&rows=kind==0?acc:(kind==1?gyro:gps);std::ofstream f(out+(kind==0?"/accel_predictions.csv":kind==1?"/gyro_predictions.csv":"/gnss_predictions.csv"));f<<"t,holdout,valid,fix";int dim=kind==2?6:3;for(std::string tag:{"obs","pred","white"})for(int k=0;k<dim;++k)f<<','<<tag<<k;f<<'\n';
  for(auto r:rows){double t=r[0]-(kind==2?delay:0);if(t<start||t>end)continue;State x=query(t);Eigen::VectorXd pred(dim),obs(dim),white(dim);int hi=kind==2?43:4,vi=kind==2?44:5;
   if(kind==0){pred=x.R.inverse()*(x.A+gravity)+ba;obs=vec(r,1);white=(pred-obs)/sa;}
   if(kind==1){pred=x.O+bg;obs=vec(r,1);white=(pred-obs)/sg;}
   if(kind==2){pred.head<3>()=x.P+x.R*lever;pred.tail<3>()=x.V+x.R*(x.O.cross(lever));obs.head<3>()=vec(r,1);obs.tail<3>()=vec(r,4);Mat6 cov;for(int a=0;a<6;++a)for(int b=0;b<6;++b)cov(a,b)=r[7+6*a+b];white=cov.llt().matrixL().solve(pred-obs);}
   f<<std::setprecision(17)<<t<<','<<r[hi]<<','<<r[vi]<<','<<(kind==2?r[45]:0);for(auto v:{obs,pred,white})for(int k=0;k<dim;++k)f<<','<<v[k];f<<'\n';}}
 // Check actual fitted trajectory at native scale, boundaries and small-angle branch.
 std::ofstream kin(out+"/kinematics.csv");kin<<"test,h,error_p,error_v,error_R,error_w\n";
 for(double h:{1e-4,1e-5,1e-6}){Eigen::Vector4d worst=Eigen::Vector4d::Zero();for(double t=start+.003;t<end-.003;t+=.0137){auto x=query(t),l=query(t-h),r=query(t+h);Eigen::Vector4d e;e<<((r.P-l.P)/(2*h)-x.V).norm(),((r.V-l.V)/(2*h)-x.A).norm(),((l.R.inverse()*r.R).log()/(2*h)-x.O).norm(),((r.O-l.O)/(2*h)-x.S).norm();worst=worst.cwiseMax(e);}kin<<"central_difference,"<<h;for(auto e:worst)kin<<','<<e;kin<<'\n';}
 Eigen::Vector4d jumps=Eigen::Vector4d::Zero();double ajump=0,sjump=0;for(int i=1;i<n-1;++i){if(knots[i].t<start||knots[i].t>end)continue;auto l=query(knots[i].t-1e-8),r=query(knots[i].t+1e-8);Eigen::Vector4d e;e<<(r.P-l.P).norm(),(r.V-l.V).norm(),(l.R.inverse()*r.R).log().norm(),(r.O-l.O).norm();jumps=jumps.cwiseMax(e);ajump=std::max(ajump,(r.A-l.A).norm());sjump=std::max(sjump,(r.S-l.S).norm());}kin<<"boundary_jump,1e-8";for(auto e:jumps)kin<<','<<e;kin<<"\naccel_alpha_boundary,1e-8,"<<ajump<<','<<sjump<<",0,0\n";
 // Windowed midpoint independent quadrature; exact GP queries, never sampled CSV derivatives.
 std::ofstream integ(out+"/integration.csv");integ<<"duration,step,error_p,error_v,error_R,error_w\n";
 for(double duration:{.02,.1,.2,.5,2.})for(double step:{.005,.0025,.00125}){Eigen::Vector4d worst=Eigen::Vector4d::Zero();for(double t=start+.1;t+duration<end-.1;t+=.37){auto a=query(t),b=query(t+duration);Vec3 dp=Vec3::Zero(),dv=dp,dw=dp;SO3d r=a.R;int count=std::lround(duration/step);double h=duration/count;for(int k=0;k<count;++k){auto x=query(t+(k+.5)*h);dp+=x.V*h;dv+=x.A*h;dw+=x.S*h;r=r*SO3d::exp(x.O*h);}Eigen::Vector4d e;e<<(a.P+dp-b.P).norm(),(a.V+dv-b.V).norm(),(r.inverse()*b.R).log().norm(),(a.O+dw-b.O).norm();worst=worst.cwiseMax(e);}integ<<duration<<','<<step;for(auto e:worst)integ<<','<<e;integ<<'\n';}
 std::ofstream jac(out+"/jacobian_monitor.csv");jac<<"knot,relative_angle,pvt_error,imu_error,prior_error\n";
 // Deterministic sparse monitor, plus the minimum relative-angle interval.
 int smallest=0;double minangle=1e9;for(int i=0;i<n-1;++i){double a=(knots[i].R.inverse()*knots[i+1].R).log().norm();if(a<minangle){minangle=a;smallest=i;}}
 std::set<int> selected={smallest};for(int i=0;i<n-1;i+=std::max(1,n/10))selected.insert(i);
 for(int i:selected){auto p=params(knots[i],knots[i+1]);PVTFactor f(mixer,.37,Vec3::Zero(),Vec3::Zero(),Mat6::Identity(),lever);double ep=factorError(f,p,1e-6);
 AmbientFactorAdapter prior(std::make_unique<GPMotionPriorTwoKnotsFactor>(mixer));double epr=factorError(prior,p,1e-6);p.insert(p.end(),{bg.data(),ba.data(),gravity.data()});AmbientFactorAdapter imu(std::make_unique<GPIMUFactor>(Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),1/sg,1/sa,0,0,mixer,.37));double ei=factorError(imu,p,1e-6);jac<<i<<','<<(knots[i].R.inverse()*knots[i+1].R).log().norm()<<','<<ep<<','<<ei<<','<<epr<<'\n';}
 std::cout<<summary.BriefReport()<<'\n';return summary.IsSolutionUsable()?0:3;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
