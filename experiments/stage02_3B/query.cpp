#include "phase_motion.hpp"
using namespace phasejoint;
int main(int argc,char**argv){try{if(argc!=4){std::cerr<<"run times.csv output.csv\n";return 2;}std::string run=argv[1];auto rows=read(run+"/knots.csv"),settings=read(run+"/query_settings.csv"),cr=read(run+"/coefficients.csv");int H=int(settings[0][0]);std::vector<double>c;for(auto r:cr)c.push_back(r[1]);PhaseCurve ph(run+"/phase_pieces.csv");std::vector<State>knots;
 for(auto r:rows){State k(r[0],SO3d(Eigen::Quaterniond(r[13],r[10],r[11],r[12])),vec(r,14),vec(r,17),vec(r,1),vec(r,4),vec(r,7));knots.push_back(k);}
 double dt=(knots.back().t-knots.front().t)/(knots.size()-1);GaussianProcess gp(dt,settings[0][4]*Mat3::Identity(),settings[0][3]*Mat3::Identity());gp.setStartTime(knots.front().t);gp.extendKnotsTo(knots.back().t,State(),false);for(size_t i=0;i<knots.size();++i)gp.setKnot(i,knots[i]);
 if(std::filesystem::exists(argv[3]))throw std::runtime_error("Refuse query overwrite");std::ofstream out(argv[3]);out<<stateHeader;for(auto r:read(argv[2])){auto s=auditedQuery(gp,r[0]);writeState(out,H?compose(s,ph.query(r[0]),c.data(),H):s);}return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
