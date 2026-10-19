#pragma once
#include "phase_motion.hpp"
namespace gnssablation {
inline Vec3 retainedPositionVelocity(const phasejoint::Rows&rows,double t){
 auto it=std::lower_bound(rows.begin(),rows.end(),t,[](const auto&r,double v){return r[0]<v;});
 size_t j=it==rows.begin()?0:std::min(size_t(it-rows.begin()-1),rows.size()-2);
 double dt=rows[j+1][0]-rows[j][0];if(dt<=0)throw std::runtime_error("Nonincreasing retained GNSS time");
 return (phasejoint::vec(rows[j+1],1)-phasejoint::vec(rows[j],1))/dt;
}
}
