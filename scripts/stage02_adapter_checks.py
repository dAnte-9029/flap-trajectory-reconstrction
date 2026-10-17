#!/usr/bin/env python3
"""Independent geometry/calibration checks using known physical special cases."""
import json
import numpy as np
from stage02_extract import ecef,ned_matrix,calibration
checks={}
def check(name,error,tolerance):
    checks[name]={'error':float(error),'tolerance':tolerance,'passed':bool(error<=tolerance)}
check('equator_ecef',np.linalg.norm(ecef(np.array([[0.,0.,0.]]))[0]-[6378137,0,0]),1e-8)
check('north_pole_ecef',np.linalg.norm(ecef(np.array([[90.,0.,0.]]))[0]-[0,0,6356752.314245179]),1e-7)
n=ned_matrix(0,0);check('equator_ned_axes',np.linalg.norm(n-np.array([[0,0,1],[0,1,0],[-1,0,0]])),1e-12)
p={'SENS_BOARD_ROT':8,'SENS_BOARD_X_OFF':0.,'SENS_BOARD_Y_OFF':0.,'SENS_BOARD_Z_OFF':0.,'TC_A_ENABLE':0,'TC_G_ENABLE':0,
 'CAL_ACC0_ID':4259850,'CAL_ACC0_ROT':-1,'CAL_ACC0_XOFF':.1,'CAL_ACC0_YOFF':.2,'CAL_ACC0_ZOFF':.3,
 'CAL_ACC0_XSCALE':2.,'CAL_ACC0_YSCALE':.5,'CAL_ACC0_ZSCALE':1.}
r,o,s=calibration(p,'accel',4259850)
check('rolled_board_static_specific_force',np.linalg.norm(r@((np.array([.1,.2,10.10665])-o)*s)-[0,0,-9.80665]),1e-12)
failed=False
try:calibration(p,'accel',12345)
except AssertionError:failed=True
check('device_id_mismatch_rejected',int(not failed),0)
# Receiver velocity field is a variance: independent scalar example.
variance=.04;floor=.15;check('velocity_sigma_not_squared_again',abs(np.sqrt(variance+floor**2)-.25),1e-12)
print(json.dumps(checks,indent=2));assert all(c['passed'] for c in checks.values())
