// Repository-local full-observation adaptation of Stage02.6; upstream unchanged.
#include "pvt_blocks.hpp"
#include "full_mode.hpp"
#include "initialization.hpp"
using namespace gnssablation;
#include <factor/GPIMUFactor.h>
#include <factor/GPMotionPriorTwoKnotsFactor.h>
using namespace phasejoint;

int main(int argc,char**argv){try{
 if(argc!=22 || std::string(argv[21])!="full_observation"){std::cerr<<"input output phase start end dt qt qr sigmaA sigmaG delay lx ly lz H maxiter initScale coeffPriorScale group traditionalInit full_observation\n";return 2;}
 std::string input=argv[1],out=argv[2];if(std::filesystem::exists(out))throw std::runtime_error("Refuse existing solve output");std::filesystem::create_directories(out);
 PhaseCurve phase(argv[3]);double start=std::stod(argv[4]),end=std::stod(argv[5]),dt=std::stod(argv[6]),qt=std::stod(argv[7]),qr=std::stod(argv[8]);
 double sa=std::stod(argv[9]),sg=std::stod(argv[10]),delay=std::stod(argv[11]);Vec3 lever(std::stod(argv[12]),std::stod(argv[13]),std::stod(argv[14]));
 int H=std::stoi(argv[15]),maxiter=std::stoi(argv[16]);double initScale=std::stod(argv[17]),priorScale=std::stod(argv[18]);
 int group=std::stoi(argv[19]);bool traditional=std::stoi(argv[20])!=0;
 if(H!=0||group<0||group>2)throw std::runtime_error("Stage02.6 permits only H0 slow GP and G0/G1/G2");
 auto acc=read(input+"/accel.csv"),gyro=read(input+"/gyro.csv"),gps=read(input+"/gnss.csv"),att=read(input+"/attitude_init.csv");
 verifyFull(acc,gyro,gps);
 double biasScale=biasPriorScale();
 double t0=start-2,finish=end+2;int n=std::lround((finish-t0)/dt)+1;std::vector<State>knots;knots.reserve(n);
 Rows gpstrain,gtrain;for(auto r:gps)if(r[43]==0&&r[44]!=0){r[0]-=delay;gpstrain.push_back(r);}for(auto r:gyro)if(r[4]==0&&r[5]!=0)gtrain.push_back(r);
 if(gpstrain.size()<4||gtrain.size()<4)throw std::runtime_error("Insufficient train-only initialization inputs");
 auto linear=[&](const Rows&rows,double t,int col){auto it=std::lower_bound(rows.begin(),rows.end(),t,[](const auto&r,double v){return r[0]<v;});size_t j=it==rows.begin()?0:std::min(size_t(it-rows.begin()-1),rows.size()-2);double s=(t-rows[j][0])/(rows[j+1][0]-rows[j][0]);return ((1-s)*vec(rows[j],col)+s*vec(rows[j+1],col)).eval();};
 auto initialR=[&](double t){auto it=std::lower_bound(att.begin(),att.end(),t,[](const auto&r,double v){return r[0]<v;});size_t j=it==att.begin()?0:std::min(size_t(it-att.begin()-1),att.size()-2);
  SO3d a(Eigen::Quaterniond(att[j][4],att[j][1],att[j][2],att[j][3])),b(Eigen::Quaterniond(att[j+1][4],att[j+1][1],att[j+1][2],att[j+1][3]));double s=(t-att[j][0])/(att[j+1][0]-att[j][0]);return a*SO3d::exp(s*(a.inverse()*b).log());};
 for(int i=0;i<n;++i){State k(t0+i*dt);k.R=initialR(k.t);k.P=linear(gpstrain,k.t,1)-k.R*lever;k.V=traditional?linear(gpstrain,k.t,4):retainedPositionVelocity(gpstrain,k.t);k.O=linear(gtrain,k.t,1);k.A.setZero();k.S.setZero();knots.push_back(k);}
 std::ofstream initial(out+"/initial_knots.csv");initial<<stateHeader;for(auto k:knots)writeState(initial,k);initial.close();
 std::vector<double>coeff(12*H,0.);for(int i=0;i<12*H;++i){int h=(i%(6*H))/6+1;double sigma=i<6*H?.05/(h*h):.05/h;coeff[i]=initScale*sigma*std::sin(i+1.);}
 Vec3 ba=Vec3::Zero(),bg=Vec3::Zero(),gravity(0,0,-9.80665);auto mixer=std::make_shared<GPMixer>(dt,qr*Mat3::Identity(),qt*Mat3::Identity());ceres::Problem problem;
 std::vector<ceres::ResidualBlockId> imuIds,gpsIds,priorIds,posIds,velIds;
 for(int i=0;i<n-1;++i)priorIds.push_back(problem.AddResidualBlock(new AmbientFactorAdapter(std::make_unique<GPMotionPriorTwoKnotsFactor>(mixer)),nullptr,params(knots[i],knots[i+1])));
 for(auto&k:knots)problem.SetManifold(k.R.data(),new AuditedSO3Manifold());
 if(H)problem.AddResidualBlock(new CoefficientPrior(H,.05*priorScale,.05*priorScale),nullptr,coeff.data());
 int skippedPhase=0,na=0,ng=0,nn=0;
 auto addimu=[&](const Rows&rows,bool isacc){for(const auto&r:rows){double t=r[0];if(r[4]!=0||r[5]==0||t<t0||t>=finish)continue;if(H && !phase.valid(t)){++skippedPhase;continue;}
  int j=std::floor((t-t0)/dt);auto p=params(knots[j],knots[j+1]);ceres::CostFunction*factor;
  if(H){p.insert(p.end(),{coeff.data(),bg.data(),ba.data()});factor=new PhaseFactor(mixer,(t-knots[j].t)/dt,phase.query(t),H,isacc?0:1,vec(r,1),Mat3::Identity()/(isacc?sa:sg));}
  else{p.insert(p.end(),{bg.data(),ba.data(),gravity.data()});factor=new AmbientFactorAdapter(std::make_unique<GPIMUFactor>(isacc?vec(r,1):Vec3::Zero(),isacc?Vec3::Zero():vec(r,1),Vec3::Zero(),Vec3::Zero(),isacc?0:1/sg,isacc?1/sa:0,0,0,mixer,(t-knots[j].t)/dt));}
  imuIds.push_back(problem.AddResidualBlock(factor,new ceres::HuberLoss(3),p));if(isacc)++na;else ++ng;}};
 addimu(acc,true);addimu(gyro,false);if(!H)problem.SetParameterBlockConstant(gravity.data());
 problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BiasPrior,3,3>(new BiasPrior{.5*biasScale}),nullptr,ba.data());
 problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BiasPrior,3,3>(new BiasPrior{.02*biasScale}),nullptr,bg.data());
 for(const auto&r:gpstrain){double t=r[0];if(t<t0||t>=finish)continue;if(H && !phase.valid(t)){++skippedPhase;continue;}int j=std::floor((t-t0)/dt);auto p=params(knots[j],knots[j+1]);Mat6 cov;
  for(int a=0;a<6;++a)for(int b=0;b<6;++b)cov(a,b)=r[7+6*a+b];ceres::CostFunction*factor;
  if(H){p.insert(p.end(),{coeff.data(),bg.data(),ba.data()});Vec6 obs;obs<<vec(r,1),vec(r,4);Mat6 W=cov.llt().matrixL().solve(Mat6::Identity());factor=new PhaseFactor(mixer,(t-knots[j].t)/dt,phase.query(t),H,2,obs,W,lever);}
  else if(group==0)factor=new PVTFactor(mixer,(t-knots[j].t)/dt,vec(r,1),vec(r,4),cov,lever);
  else {
   auto *position=positionFactor(mixer,(t-knots[j].t)/dt,vec(r,1),cov,lever);
   posIds.push_back(problem.AddResidualBlock(position,new ceres::HuberLoss(3),p));
   if(group==1)velIds.push_back(problem.AddResidualBlock(new PVTBlock(mixer,(t-knots[j].t)/dt,vec(r,1),vec(r,4),cov,lever,PVTBlock::ConditionalVelocity),new ceres::HuberLoss(3),p));
   ++nn;continue;
  }
  gpsIds.push_back(problem.AddResidualBlock(factor,new ceres::HuberLoss(3),p));++nn;}
 ceres::Solver::Options opts;opts.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;opts.max_num_iterations=maxiter;opts.num_threads=2;opts.function_tolerance=1e-8;opts.gradient_tolerance=1e-10;opts.parameter_tolerance=1e-9;opts.minimizer_progress_to_stdout=true;
 // Eliminate slow knot variables before the few global periodic coefficients.
 // Ceres chooses a fill-reducing ordering; do not create a new GP solver.
 ceres::Solver::Summary summary;ceres::Solve(opts,&problem,&summary);
 std::ofstream solver(out+"/solver.txt");solver<<summary.FullReport();
 std::ofstream status(out+"/solver.csv");status<<"usable,termination,initial_cost,final_cost,iterations,time_s,local_parameters,accel_factors,gyro_factors,gnss_factors,phase_boundary_excluded,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z\n"<<std::setprecision(17)<<summary.IsSolutionUsable()<<','<<int(summary.termination_type)<<','<<summary.initial_cost<<','<<summary.final_cost<<','<<summary.iterations.size()<<','<<summary.total_time_in_seconds<<','<<(18*n+6+12*H)<<','<<na<<','<<ng<<','<<nn<<','<<skippedPhase;
 for(auto v:{ba,bg})for(int j=0;j<3;++j)status<<','<<v[j];status<<'\n';
 std::ofstream cp(out+"/coefficients.csv");cp<<"index,value,prior_sigma\n"<<std::setprecision(17);for(int i=0;i<12*H;++i){int h=(i%(6*H))/6+1;cp<<i<<','<<coeff[i]<<','<<(i<6*H?.05/(h*h):.05/h)*priorScale<<'\n';}
 std::ofstream setting(out+"/query_settings.csv");setting<<"H,start,end,qt,qr,delay,lx,ly,lz\n"<<std::setprecision(17)<<H<<','<<start<<','<<end<<','<<qt<<','<<qr<<','<<delay<<','<<lever.x()<<','<<lever.y()<<','<<lever.z()<<'\n';
 std::filesystem::copy_file(argv[3],out+"/phase_pieces.csv");
 GaussianProcess gp(mixer);gp.setStartTime(t0);gp.extendKnotsTo(knots.back().t,State(),false);for(int i=0;i<n;++i)gp.setKnot(i,knots[i]);
 auto slow=[&](double t){return auditedQuery(gp,t);};auto query=[&](double t){auto s=slow(t);return H?compose(s,phase.query(t),coeff.data(),H):s;};
 std::ofstream ko(out+"/knots.csv"),dense(out+"/trajectory.csv"),sdense(out+"/slow_trajectory.csv");ko<<stateHeader;dense<<stateHeader;sdense<<stateHeader;for(auto k:knots)writeState(ko,k);
 for(int i=0;i<=std::lround((end-start)/.005);++i){double t=start+i*.005;writeState(dense,query(t));writeState(sdense,slow(t));}
 for(int kind=0;kind<3;++kind){const auto&rows=kind==0?acc:kind==1?gyro:gps;std::ofstream f(out+(kind==0?"/accel_predictions.csv":kind==1?"/gyro_predictions.csv":"/gnss_predictions.csv"));int dim=kind==2?6:3;f<<"t,holdout,valid,fix";for(std::string label:{"obs","pred","white","slowpred"})for(int j=0;j<dim;++j)f<<','<<label<<j;f<<'\n';
  for(auto r:rows){double t=r[0]-(kind==2?delay:0);if(t<t0||t>=finish)continue;if(H && !phase.valid(t))throw std::runtime_error("Target phase coverage failure");auto total=query(t),s=slow(t);auto pred=predicted(total,kind,lever),spred=predicted(s,kind,lever);Eigen::VectorXd obs(dim),white(dim);int hi=kind==2?43:4,vi=kind==2?44:5;
   if(kind<2){pred+=kind==0?ba:bg;spred+=kind==0?ba:bg;obs=vec(r,1);white=(pred-obs)/(kind==0?sa:sg);}else{obs.head<3>()=vec(r,1);obs.tail<3>()=vec(r,4);Mat6 cov;for(int a=0;a<6;++a)for(int b=0;b<6;++b)cov(a,b)=r[7+6*a+b];white=cov.llt().matrixL().solve(pred-obs);}
   f<<std::setprecision(17)<<t<<','<<r[hi]<<','<<r[vi]<<','<<(kind==2?r[45]:0);for(auto v:{obs,pred,white,spred})for(int j=0;j<dim;++j)f<<','<<v[j];f<<'\n';}}
 std::ofstream costs(out+"/cost_groups.csv");costs<<"group,robust_cost\n";for(auto group:{std::make_pair("imu",imuIds),std::make_pair("gnss",gpsIds),std::make_pair("slow_prior",priorIds),std::make_pair("gnss_position",posIds),std::make_pair("gnss_conditional_velocity",velIds)}){ceres::Problem::EvaluateOptions eo;eo.residual_blocks=group.second;double cost=0;if(!group.second.empty())problem.Evaluate(eo,&cost,nullptr,nullptr,nullptr);costs<<group.first<<','<<cost<<'\n';}
 std::ofstream counts(out+"/gnss_components.csv");counts<<"group,traditional_initialization,joint,position,conditional_velocity\n"<<group<<','<<traditional<<','<<gpsIds.size()<<','<<posIds.size()<<','<<velIds.size()<<'\n';
 std::ofstream kin(out+"/kinematics.csv");kin<<"test,h,p,v,R,omega,a,alpha\n";
 for(double h:{1e-4,1e-5,1e-6}){Eigen::Vector4d worst=Eigen::Vector4d::Zero();for(double t=start+.003173;t<end-.003;t+=.01373){auto x=query(t),l=query(t-h),r=query(t+h);Eigen::Vector4d e;e<<((r.P-l.P)/(2*h)-x.V).norm(),((r.V-l.V)/(2*h)-x.A).norm(),((l.R.inverse()*r.R).log()/(2*h)-x.O).norm(),((r.O-l.O)/(2*h)-x.S).norm();worst=worst.cwiseMax(e);}kin<<"derivative,"<<h;for(double v:worst)kin<<','<<v;kin<<",0,0\n";}
 for(double h:{1e-7,1e-9,1e-11}){Eigen::Matrix<double,6,1>worst=Eigen::Matrix<double,6,1>::Zero();for(int i=1;i<n-1;++i){double t=knots[i].t;if(t<t0||t>=finish)continue;auto l=query(t-h),r=query(t+h);Eigen::Matrix<double,6,1>e;e<<(r.P-l.P).norm(),(r.V-l.V).norm(),(l.R.inverse()*r.R).log().norm(),(r.O-l.O).norm(),(r.A-l.A).norm(),(r.S-l.S).norm();worst=worst.cwiseMax(e);}kin<<"boundary,"<<h;for(double v:worst)kin<<','<<v;kin<<'\n';}
 std::ofstream integ(out+"/integration.csv");integ<<"duration,step,p,v,R,omega,left_endpoint_v,left_endpoint_omega\n";
 for(double duration:{.02,.05,.1,.2})for(double step:{.005,.00125,.0003125}){Eigen::Matrix<double,6,1>worst=Eigen::Matrix<double,6,1>::Zero();for(double t=start+.10317;t+duration<end-.1;t+=.3713){auto a=query(t),b=query(t+duration);Vec3 dp=Vec3::Zero(),dv=dp,dw=dp;SO3d R=a.R;int count=std::ceil(duration/step);double h=duration/count;for(int j=0;j<count;++j){auto x=query(t+(j+.5)*h);dp+=x.V*h;dv+=x.A*h;dw+=x.S*h;R=R*SO3d::exp(x.O*h);}Eigen::Matrix<double,6,1>e;e<<(a.P+dp-b.P).norm(),(a.V+dv-b.V).norm(),(R.inverse()*b.R).log().norm(),(a.O+dw-b.O).norm(),(a.V+a.A*duration-b.V).norm(),(a.O+a.S*duration-b.O).norm();worst=worst.cwiseMax(e);}integ<<duration<<','<<step;for(double v:worst)integ<<','<<v;integ<<'\n';}
 std::cout<<summary.BriefReport()<<'\n';return summary.IsSolutionUsable()?0:3;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
