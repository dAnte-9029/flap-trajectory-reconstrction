// Post-fit verifier: reads saved knot states and calls the SAME upstream query.
// Runs no optimization and never modifies fitted trajectories.
#include "query_adapter.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
using State=GPState<double>;
int main(int argc,char**argv){
 if(argc!=2)return 2;std::ifstream f(std::string(argv[1])+"/knots.csv");if(!f)return 2;std::string line;std::getline(f,line);std::vector<State> k;
 while(std::getline(f,line)){std::stringstream ss(line);std::string v;std::vector<double>d;while(std::getline(ss,v,','))d.push_back(std::stod(v));State x(d[0]);x.P=Vec3(d[1],d[2],d[3]);x.V=Vec3(d[4],d[5],d[6]);x.A=Vec3(d[7],d[8],d[9]);x.R=SO3d(Eigen::Quaterniond(d[13],d[10],d[11],d[12]));x.O=Vec3(d[14],d[15],d[16]);x.S=Vec3(d[17],d[18],d[19]);k.push_back(x);}
 double dt=(k.back().t-k.front().t)/(k.size()-1);GaussianProcess gp(dt,Mat3::Identity(),Mat3::Identity());gp.setStartTime(k.front().t);gp.extendKnotsTo(k.back().t,State(),false);for(size_t i=0;i<k.size();++i)gp.setKnot(i,k[i]);
 std::cout<<"test,h,p,v,a,R,omega,alpha\n"<<std::setprecision(17);
 for(double h:{1e-7,1e-8,1e-9,1e-10,1e-11}){Eigen::Matrix<double,6,1> e=Eigen::Matrix<double,6,1>::Zero();for(size_t i=1;i+1<k.size();++i){auto l=auditedQuery(gp,k[i].t-h),r=auditedQuery(gp,k[i].t+h);Eigen::Matrix<double,6,1>d;d<<(r.P-l.P).norm(),(r.V-l.V).norm(),(r.A-l.A).norm(),(l.R.inverse()*r.R).log().norm(),(r.O-l.O).norm(),(r.S-l.S).norm();e=e.cwiseMax(d);}std::cout<<"boundary_jump,"<<h;for(double v:e)std::cout<<','<<v;std::cout<<'\n';}
 for(double h:{1e-5,1e-6,1e-7}){Eigen::Vector4d e=Eigen::Vector4d::Zero();for(size_t i=0;i+1<k.size();++i)for(double s:{.001,.019,.37,.81,.999}){double t=k[i].t+s*dt;auto x=auditedQuery(gp,t),l=auditedQuery(gp,t-h),r=auditedQuery(gp,t+h);Eigen::Vector4d d;d<<((r.P-l.P)/(2*h)-x.V).norm(),((r.V-l.V)/(2*h)-x.A).norm(),((l.R.inverse()*r.R).log()/(2*h)-x.O).norm(),((r.O-l.O)/(2*h)-x.S).norm();e=e.cwiseMax(d);}std::cout<<"interior_derivatives,"<<h<<','<<e[0]<<','<<e[1]<<",0,"<<e[2]<<','<<e[3]<<",0\n";}
 return 0;
}
