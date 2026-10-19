"""A serializable C2 phase curve using SciPy's existing smoothing-spline solver."""
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline, PPoly, make_smoothing_spline


class ContinuousPhase:
    def __init__(self, spline, origin, offset, slope, domain, core, metadata=None):
        self.spline = spline
        self.origin = float(origin)
        self.offset = float(offset)
        self.slope = float(slope)
        self.domain = np.asarray(domain, float)
        self.core = np.asarray(core, float)
        self.metadata = metadata or {}

    @classmethod
    def fit(cls, t, phase, scale_hz, core, train=None, gap_limit=.03):
        t, phase = np.asarray(t, float), np.asarray(phase, float)
        if len(t) < 6 or not np.all(np.isfinite(t)) or not np.all(np.isfinite(phase)):
            raise ValueError('Need at least six finite observations; segment invalid data first')
        dt = np.diff(t)
        if np.any(dt <= 0) or np.any(dt > gap_limit) or np.any(np.diff(phase) <= 0):
            raise ValueError('Input must be contiguous and positive-rotation; segment anomalies first')
        if train is None:
            train = np.ones(len(t), bool)
        train = np.asarray(train, bool)
        if not train[0] or not train[-1] or sum(train) < 6:
            raise ValueError('Holdout fit requires both boundary anchors and six training points')
        x = t-t[0]
        slope = (phase[-1]-phase[0]) / x[-1]
        # Detrending improves numerical conditioning; the removed affine term
        # has exactly zero curvature, so it does not change the objective.
        lam = 1 / (np.median(dt) * (2*np.pi*scale_hz)**4)
        spl = make_smoothing_spline(x[train], (phase-phase[0]-slope*x)[train], lam=lam)
        return cls(spl, t[0], phase[0], slope, [t[0], t[-1]], core,
                   {'scale_hz': scale_hz, 'lambda': lam, 'training_rows': int(sum(train)),
                    'gauge': 'One fixed initial Hall-indexed phase; later resets diagnostic only',
                    'acceleration_confidence': 'candidate; sensitivity is not calibrated uncertainty'})

    def query(self, t, derivative=0):
        t = np.asarray(t, float)
        if derivative not in (0, 1, 2):
            raise ValueError('Supported derivative orders: 0, 1, 2')
        if np.any(~np.isfinite(t)) or np.any(t < self.domain[0]) or np.any(t > self.domain[1]):
            raise ValueError('No extrapolation or queries outside valid segment')
        x = t-self.origin
        value = self.spline(x, nu=derivative)
        return value + (self.offset+self.slope*x if derivative == 0 else self.slope if derivative == 1 else 0)

    def flags(self, t):
        t = np.asarray(t, float)
        valid = np.isfinite(t) & (t >= self.domain[0]) & (t <= self.domain[1])
        return {'valid': valid, 'interior_derivative_support': valid & (t >= self.core[0]) & (t <= self.core[1]),
                'absolute_zero_calibrated': np.zeros(t.shape, bool),
                'acceleration_physically_validated': np.zeros(t.shape, bool)}

    def minimum_velocity(self):
        """Check extrema of every quadratic velocity piece, not just a query grid."""
        pp = PPoly.from_spline(self.spline.derivative(1))
        candidates = []
        for i, width in enumerate(np.diff(pp.x)):
            if width <= 0 or pp.x[i] < 0 or pp.x[i+1] > self.domain[1]-self.origin:
                continue
            a, b, _ = pp.c[:, i]
            u = [0., width]
            if a != 0 and 0 < -b/(2*a) < width:
                u.append(-b/(2*a))
            candidates.extend(pp.x[i]+np.array(u))
        values = self.spline(np.array(candidates), nu=1)+self.slope
        return float(np.min(values))

    def save(self, path):
        Path(path).write_text(json.dumps(dict(knots=self.spline.t.tolist(), coefficients=self.spline.c.tolist(),
            degree=int(self.spline.k), origin=self.origin, offset=self.offset, slope=self.slope,
            domain=self.domain.tolist(), core=self.core.tolist(), metadata=self.metadata), indent=2)+'\n')

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text())
        return cls(BSpline(d['knots'], d['coefficients'], d['degree'], extrapolate=False),
                   d['origin'], d['offset'], d['slope'], d['domain'], d['core'], d['metadata'])
