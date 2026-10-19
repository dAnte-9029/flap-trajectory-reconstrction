#include "pvt_blocks.hpp"
#include "initialization.hpp"
#include <random>
using namespace gnssablation;
struct Eval {Eigen::VectorXd r;std::vector<RowMat> j;};
Eval evaluate(ceres::CostFunction&f,std::vector<double*>&p){Eval e;e.r.resize(f.num_residuals());std::vector<double*>ptr;for(auto n:f.parameter_block_sizes())e.j.emplace_back(f.num_residuals(),n);for(auto&j:e.j)ptr.push_back(j.data());f.Evaluate(p.data(),e.r.data(),ptr.data());return e;}
// Persist every checked tangent-space derivative, including SO(3) columns.
void dumpJacobian(std::ostream&out,int covariance,double fraction,const char*kind,ceres::CostFunction&f,std::vector<double*>p){
 auto e=evaluate(f,p);double h=1e-6;Eigen::VectorXd rp(f.num_residuals()),rm(f.num_residuals());
 for(int k=0;k<int(p.size());++k){int n=f.parameter_block_sizes()[k];Eigen::VectorXd saved=Eigen::Map<Eigen::VectorXd>(p[k],n);Eigen::MatrixXd J=e.j[k];
  if(n==4){Eigen::Matrix<double,4,3,Eigen::RowMajor>P;AuditedSO3Manifold().PlusJacobian(p[k],P.data());J=J*P;}
  for(int c=0;c<(n==4?3:n);++c){for(int sign:{-1,1}){Eigen::Map<Eigen::VectorXd>(p[k],n)=saved;if(n==4){Eigen::Map<SO3d>q(p[k]);Vec3 v=Vec3::Zero();v[c]=sign*h;q=q*SO3d::exp(v);}else p[k][c]+=sign*h;f.Evaluate(p.data(),sign>0?rp.data():rm.data(),nullptr);}
   auto num=((rp-rm)/(2*h)).eval();for(int r=0;r<f.num_residuals();++r)out<<covariance<<','<<fraction<<','<<kind<<','<<k<<','<<r<<','<<c<<','<<J(r,c)<<','<<num[r]<<'\n';
  }Eigen::Map<Eigen::VectorXd>(p[k],n)=saved;
 }
}
int main(int argc,char**argv){try{
 if(argc!=2)throw std::runtime_error("output.csv");if(std::filesystem::exists(argv[1]))throw std::runtime_error("Refuse overwrite");
 std::ofstream details(std::string(argv[1])+".jacobians.csv");details<<std::setprecision(17)<<"covariance,fraction,kind,block,row,column,analytic,numerical\n";
 std::mt19937 rng(26019);std::normal_distribution<double> normal;double normerror=0,costerror=0,gradienterror=0,fd=0,identity=0,minSchur=1e9,maxCond=0;int residualTests=0;
 auto mixer=std::make_shared<GPMixer>(.05,1000*Mat3::Identity(),10000*Mat3::Identity());
 State a(0,SO3d::exp(Vec3(.2,-.1,.3)),Vec3(.7,-.4,.2),Vec3(1.,-.3,.2),Vec3(2,1,-3),Vec3(4,-2,1),Vec3(2,3,-1));
 State b=a;b.t=.05;b.R=a.R*SO3d::exp(Vec3(.03,-.02,.011));b.P+=a.V*.05;b.V+=a.A*.05;b.O+=a.S*.05;
 auto p=params(a,b);ceres::HuberLoss loss(3);
 for(int c=0;c<32;++c){Mat6 L=Mat6::Zero();for(int i=0;i<6;++i)for(int j=0;j<=i;++j)L(i,j)=i==j?.2+std::abs(normal(rng)):.3*normal(rng);Mat6 cov=L*L.transpose();
  if(c==31){cov=Mat6::Identity();cov.topRightCorner<3,3>()=.9999*Mat3::Identity();cov.bottomLeftCorner<3,3>()=.9999*Mat3::Identity();}
  PVTWhitening w(cov);Eigen::SelfAdjointEigenSolver<Mat6> eig(cov);maxCond=std::max(maxCond,eig.eigenvalues().maxCoeff()/eig.eigenvalues().minCoeff());minSchur=std::min(minSchur,Eigen::SelfAdjointEigenSolver<Mat3>(w.conditional).eigenvalues().minCoeff());Mat6 W=Eigen::LLT<Mat6>(cov).matrixL().solve(Mat6::Identity());
  for(int k=0;k<64;++k){Vec6 e;for(int j=0;j<6;++j)e[j]=std::pow(10.,k%7-3)*normal(rng);double x=(W*e).squaredNorm(),y=(w.W*e).squaredNorm();normerror=std::max(normerror,std::abs(x-y)/(1+x));++residualTests;}
  if(c>5&&c<31)continue;
  for(double s:{.0001,.37,.9999}){Vec3 lever(.2,-.07,.11),obsP(1.7,1.4,-2.6),obsV(3.1,-1.3,.7);PVTFactor original(mixer,s,obsP,obsV,cov,lever);PVTBlock combined(mixer,s,obsP,obsV,cov,lever,PVTBlock::Combined),position(mixer,s,obsP,obsV,cov,lever,PVTBlock::Position),velocity(mixer,s,obsP,obsV,cov,lever,PVTBlock::ConditionalVelocity);
   auto o=evaluate(original,p),z=evaluate(combined,p);double ro[3],rz[3];loss.Evaluate(o.r.squaredNorm(),ro);loss.Evaluate(z.r.squaredNorm(),rz);costerror=std::max(costerror,std::abs(ro[0]-rz[0])/(1+ro[0]));
   for(int j=0;j<12;++j){auto go=(ro[1]*o.j[j].transpose()*o.r).eval(),gz=(rz[1]*z.j[j].transpose()*z.r).eval();gradienterror=std::max(gradienterror,(go-gz).norm()/(1+go.norm()));}
   dumpJacobian(details,c,s,"position",position,p);dumpJacobian(details,c,s,"conditional_velocity",velocity,p);
   fd=std::max({fd,jacobianError(position,p,1e-6),jacobianError(velocity,p,1e-6)});
   std::unique_ptr<PVTBlock>g1(positionFactor(mixer,s,obsP,cov,lever)),g2(positionFactor(mixer,s,obsP,cov,lever));auto x=evaluate(*g1,p),y=evaluate(*g2,p);double rho1[3],rho2[3];ceres::HuberLoss(3).Evaluate(x.r.squaredNorm(),rho1);ceres::HuberLoss(3).Evaluate(y.r.squaredNorm(),rho2);for(int j=0;j<3;++j)identity=std::max(identity,std::abs(rho1[j]-rho2[j]));identity=std::max(identity,(x.r-y.r).norm());for(int j=0;j<12;++j)identity=std::max(identity,(x.j[j]-y.j[j]).norm());identity=std::max(identity,(g1->whitening()-g2->whitening()).norm());
   PVTBlock poison(mixer,s,obsP,Vec3::Constant(1e9),cov,lever,PVTBlock::Position);auto pp=evaluate(poison,p);identity=std::max(identity,(pp.r-x.r).norm());
  }
 }
 int rejected=0;for(int k=0;k<4;++k){Mat6 c=Mat6::Identity();if(k==0)c(0,0)=-1;if(k==1)c(0,0)=0;if(k==2)c(0,1)=.1;if(k==3)c(0,0)=std::numeric_limits<double>::quiet_NaN();try{PVTWhitening w(c);}catch(...){++rejected;}}
 Rows init{{0,0,0,0,100,200,300},{.2,1,.3,.4,100,200,300},{.6,2,1,-.2,100,200,300}};auto poison=init;for(auto&r:poison)for(int j=4;j<7;++j)r[j]+=1e9;double initerr=0;for(double t:{-.1,.1,.3,.8})initerr=std::max(initerr,(retainedPositionVelocity(init,t)-retainedPositionVelocity(poison,t)).norm());
 double joint[3],separate[3];loss.Evaluate(32,joint);loss.Evaluate(16,separate);double huberdiff=std::abs(.5*joint[0]-separate[0]);
 bool pass=normerror<1e-8&&costerror<1e-8&&gradienterror<1e-8&&fd<1e-5&&identity==0&&initerr==0&&rejected==4&&huberdiff>1;
 std::ofstream out(argv[1]);out<<"passed,residual_tests,whitening_norm_relative,combined_huber_relative,combined_gradient_relative,finite_difference_max,position_identity_max,velocity_poison_init_max,rejected_invalid,min_schur_eigenvalue,max_cov_condition,joint_vs_separate_huber_difference\n"<<std::setprecision(17)<<pass<<','<<residualTests<<','<<normerror<<','<<costerror<<','<<gradienterror<<','<<fd<<','<<identity<<','<<initerr<<','<<rejected<<','<<minSchur<<','<<maxCond<<','<<huberdiff<<'\n';std::cout<<"passed="<<pass<<" FD="<<fd<<" parity="<<gradienterror<<'\n';return pass?0:1;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
