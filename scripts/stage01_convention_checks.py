#!/usr/bin/env python3
"""Executed mathematical checks of the PROPOSED observation conventions.

These tests do not import or validate GPTR/gnssFGO binaries. They use independent
analytic rigid-body trajectories, not raw logs or reconstructed labels.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def state(t):
    # Noncommuting orientation R=Rx(a) Ry(b), body angular derivatives in rad/s.
    w = 8 * np.pi
    a, b = .08*np.sin(w*t), .06*np.cos(w*t)
    ad, bd = .08*w*np.cos(w*t), -.06*w*np.sin(w*t)
    add, bdd = -w*w*a, -w*w*b
    rx = Rotation.from_rotvec([a, 0, 0]).as_matrix()
    ry = Rotation.from_rotvec([0, b, 0]).as_matrix()
    u = ry.T @ np.array([ad, 0, 0])
    omega = u + [0, bd, 0]
    alpha = ry.T @ np.array([add, 0, 0]) - np.cross([0, bd, 0], u) + [0, bdd, 0]
    p = np.array([.03*np.sin(w*t), .02*np.cos(w*t), .1*t])
    v = np.array([.03*w*np.cos(w*t), -.02*w*np.sin(w*t), .1])
    acc = np.array([-.03*w*w*np.sin(w*t), -.02*w*w*np.cos(w*t), 0])
    return rx @ ry, omega, alpha, p, v, acc


def run():
    checks = []

    def check(name, error, tol):
        checks.append(dict(name=name, error=float(error), tolerance=tol,
                           passed=bool(np.isfinite(error) and error <= tol)))

    t, h = .137, 1e-5
    r, omega, alpha, p, v, acc = state(t)
    rp, wp, _, pp, vp, _ = state(t+h)
    rm, wm, _, pm, vm, _ = state(t-h)
    check('analytic_p_derivative', np.linalg.norm((pp-pm)/(2*h)-v), 1e-6)
    check('analytic_v_derivative', np.linalg.norm((vp-vm)/(2*h)-acc), 1e-6)
    check('analytic_R_body_omega', np.linalg.norm(Rotation.from_matrix(rm.T@rp).as_rotvec()/(2*h)-omega), 1e-6)
    check('analytic_omega_body_alpha', np.linalg.norm((wp-wm)/(2*h)-alpha), 1e-6)
    # A deliberately nonzero antenna lever arm: exposes an omitted velocity term.
    lever = np.array([.2, -.1, .05])
    predicted_v = v + r @ np.cross(omega, lever)
    numeric_v = ((pp+rp@lever)-(pm+rm@lever))/(2*h)
    check('antenna_velocity_lever_arm', np.linalg.norm(predicted_v-numeric_v), 1e-6)
    predicted_a = acc + r @ (np.cross(alpha, lever)+np.cross(omega,np.cross(omega,lever)))
    numeric_a = ((pp+rp@lever)-2*(p+r@lever)+(pm+rm@lever))/h**2
    check('rigid_point_acceleration_lever_arm', np.linalg.norm(predicted_a-numeric_a), 2e-5)
    gravity = np.array([0, 0, 9.80665])
    bias = np.array([.1, -.2, .05])
    measured = r.T @ (predicted_a-gravity) + bias
    recovered = r @ (measured-bias)+gravity
    check('ned_specific_force_to_imu_acceleration', np.linalg.norm(recovered-predicted_a), 1e-12)
    check('stationary_ned_specific_force', np.linalg.norm(-gravity-[0,0,-9.80665]), 0)
    # Driver-specific variance semantics: NOT squaring the velocity variance again.
    eph, epv, velocity_variance = .02, .04, .09
    covariance = np.diag([eph**2, eph**2, epv**2]+[velocity_variance]*3)
    whitening = np.linalg.inv(np.linalg.cholesky(covariance))
    check('pvt_diagonal_surrogate_whitening', np.linalg.norm(whitening@covariance@whitening.T-np.eye(6)), 1e-12)
    check('velocity_variance_is_not_stddev', abs(whitening[3,3]-1/.3), 1e-12)
    # Show that replacing async acquisition time with the nearby accel time changes gyro.
    offset = .00205  # Stage 00 observed nearest-sample P95 scale, not absolute latency.
    async_change = np.linalg.norm(state(t+offset)[1]-omega)
    return dict(scope='local analytic convention checks only; neither upstream binary executed',
                checks=checks, asynchronous_2_05ms_gyro_change_rad_s=float(async_change),
                omitted_velocity_lever_arm_error_m_s=float(np.linalg.norm(predicted_v-v)),
                all_passed=all(c['passed'] for c in checks))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='runs/stage01_20261017/convention_checks.json')
    args = parser.parse_args()
    result = run()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['all_passed'] else 1)
