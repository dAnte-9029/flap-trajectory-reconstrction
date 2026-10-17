#!/usr/bin/env python3
"""Pin local upstream/PX4 sources reviewed at Stage 00 without changing them."""
from pathlib import Path
import json
import subprocess
import stage00_audit as a

cfg=json.loads((a.ROOT/'configs/stage00_audit.json').read_text());audit=a.ROOT/cfg['output'];root=audit.parent
out=root/'provenance_final';out.mkdir(exist_ok=False)
metas=[json.loads(p.read_text()) for p in (audit/'logs').glob('*.json')]
assert len(metas)==json.loads((audit/'manifest.json').read_text())['counts']['unique_ulogs']
px4paths=['src/drivers/uavcan/sensors/gnss.cpp','src/drivers/uavcan/sensors/gnss_relative.cpp','src/modules/sensors/vehicle_gps_position/PpsTimeSync.cpp','msg/SensorAccel.msg','msg/SensorGyro.msg','msg/SensorCombined.msg','msg/VehicleImu.msg','msg/VehicleImuStatus.msg','msg/VehicleAcceleration.msg','msg/SensorGps.msg','msg/SensorGnssRelative.msg','msg/versioned/VehicleAttitude.msg','msg/versioned/VehicleAngularVelocity.msg','msg/versioned/VehicleLocalPosition.msg','msg/versioned/VehicleGlobalPosition.msg','msg/WingPhase.msg','msg/FlapFrequency.msg','msg/EncoderCount.msg','msg/HallEvent.msg','src/modules/wing_phase/WingPhase.cpp','src/drivers/encoder/as5600/AS5600.cpp','src/lib/drivers/accelerometer/PX4Accelerometer.cpp','src/lib/drivers/gyroscope/PX4Gyroscope.cpp','src/modules/sensors/vehicle_acceleration/VehicleAcceleration.cpp','src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp','src/modules/sensors/vehicle_imu/VehicleIMU.cpp','src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp','src/modules/sensors/vehicle_gps_position/gps_blending.cpp','src/modules/logger/logged_topics.cpp']
upstream={
'gptr':['LICENSE','README.md','gptr/include/GaussianProcess.hpp','gptr/include/SE3JQ.hpp','gptr/include/factor/GPMotionPriorTwoKnotsFactor.h','gptr/include/factor/GPIMUFactor.h','gptr/src/GPUI.cpp','gptr/src/GPUI.hpp','gptr/src/GPVICalib.cpp','gptr/CMakeLists.txt','gptr/package.xml'],
'gnssFGO':['LICENSE','README.md','.gitmodules','online_fgo/include/factor/gnss/GPInterpolatedPVTFactor.h','online_fgo/include/factor/gnss/PVTFactor.h','online_fgo/include/factor/gnss/GPInterpolatedGPSFactor.h','online_fgo/include/model/gp_interpolator/GPWNOJInterpolator.h','online_fgo/include/model/gp_interpolator/GPWNOJInterpolatorFull.h','online_fgo/include/factor/motion/GPWNOJPrior.h','online_fgo/include/factor/motion/GPWNOJPriorFull.h','online_fgo/src/graph/GraphTimeCentric.cpp','online_fgo/src/gnss_fgo/GNSSFGOLocalizationBase.cpp','online_fgo/src/integrator/GNSSLCIntegrator.cpp','online_fgo/src/integrator/IMUPreIntegrator.cpp','online_fgo/CMakeLists.txt']}
rows=[]
for rev in sorted(set(m['info']['ver_sw'] for m in metas)):
 for path in px4paths:
  proc=subprocess.run(['git','-C','/home/zn/PX4-Autopilot','show',f'{rev}:{path}'],capture_output=True)
  target=out/'px4'/rev/path
  if proc.returncode==0:target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(proc.stdout)
  rows.append(dict(repository='/home/zn/PX4-Autopilot',revision=rev,path=path,available=proc.returncode==0,sha256=a.sha(target) if proc.returncode==0 else None))
for name,paths in upstream.items():
 repo=Path('/home/zn')/name;rev=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()
 for path in paths:
  p=repo/path;rows.append(dict(repository=str(repo),revision=rev,path=path,available=p.exists(),sha256=a.sha(p) if p.exists() else None))
a.write_csv(out/'reviewed_sources.csv',rows)
a.dump(out/'manifest.json',dict(code_sha256=a.sha(Path(__file__)),firmware_log_counts={rev:sum(m['info']['ver_sw']==rev for m in metas) for rev in sorted(set(m['info']['ver_sw'] for m in metas))},sources=rows,scope='Read-only local pinned code and papers; no compilation or dependencies installed; firmware dirty-build provenance beyond recorded hash is unknown'))
print('sources',len(rows),'missing',sum(not r['available'] for r in rows))
