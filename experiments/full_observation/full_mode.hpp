#pragma once
#include "phase_motion.hpp"
#include <cstdlib>
inline void verifyFull(const phasejoint::Rows&a,const phasejoint::Rows&g,const phasejoint::Rows&n){
 for(const auto* rows:{&a,&g,&n})for(const auto&r:*rows){
  int h=rows==&n?43:4, v=h+1;
  if(r[h]!=0)throw std::runtime_error("full_observation rejects ALL residual holdout flags");
  if(r[v]!=0)for(double x:r)if(!std::isfinite(x))throw std::runtime_error("Nonfinite valid observation");
 }
}
inline double biasPriorScale(){const char*s=std::getenv("FO_BIAS_SCALE");double v=s?std::stod(s):1.;if(!(v>0))throw std::runtime_error("bias scale");return v;}
