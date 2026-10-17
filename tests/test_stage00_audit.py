import importlib.util
from pathlib import Path
import tempfile
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('audit',Path(__file__).parents[1]/'scripts/stage00_audit.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)

class AuditChecks(unittest.TestCase):
    def test_unsigned_timestamp_regression_not_underflow(self):
        d={'timestamp':np.array([1000000,1010000,1005000],dtype=np.uint64)}
        self.assertLess(np.diff(a.time_data(d))[-1],0)
    def test_gap_and_invalid_samples_not_bridged(self):
        t=np.array([1.,2.,3.,20.,21.,22.])
        np.testing.assert_allclose(a.intervals(t),[[1,3],[20,22]])
        np.testing.assert_allclose(a.intervals(t,[1,0,1,1,1,1]),[[20,22]])
    def test_union_no_double_count_and_intersection(self):
        merged=a.merge([[1,3],[2,4],[7,8]])
        self.assertEqual(a.seconds(merged),4)
        np.testing.assert_allclose(a.intersect(merged,np.array([[2,7.5]])),[[2,4],[7,7.5]])
    def test_short_increment_is_not_logging_period(self):
        t=np.array([1.,1.5,2.]);duration=.005
        self.assertAlmostEqual(a.seconds(a.merge(np.column_stack((t-duration,t)))),.015)
    def test_sparse_event_not_extrapolated_to_log(self):
        self.assertEqual(a.seconds(a.intervals([10])),0)
    def test_reset_removed_even_with_valid_state_flags(self):
        d={k:np.ones(4) for k in ['timestamp','x','y','z','vx','vy','vz','xy_valid','z_valid','v_xy_valid','v_z_valid']}
        d['xy_reset_counter']=np.array([0,0,1,1])
        mask,_=a.mask_for('vehicle_local_position',d)
        self.assertEqual(mask.tolist(),[True,True,False,True])
    def test_discovery_alternate_dates_duplicate_and_appledouble(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name in ['2026.9.6','9.19-3数据','other']:(root/name).mkdir()
            (root/'2026.9.6/a.ulg').write_bytes(a.MAGIC+b'xx')
            (root/'9.19-3数据/b.ulg').write_bytes(a.MAGIC+b'xx')
            (root/'9.19-3数据/._b.ulg').write_bytes(b'\x00\x05\x16\x07xxxx')
            (root/'other/log_2026-9-30.bin').write_bytes(a.MAGIC+b'yy')
            rows=a.discovery(root)
            self.assertEqual(len(rows),4)
            self.assertTrue(all(r['september_candidate'] for r in rows))
            self.assertEqual(sum(r['kind']=='ulog' for r in rows),3)
            self.assertEqual(len({r['sha256'] for r in rows if r['kind']=='ulog'}),2)
    def test_no_finite_values_recorded_as_missing(self):
        self.assertEqual(a.stats([float('nan')])['finite_fraction'],0)
    def test_zero_sample_clock_falls_back_but_partial_zeros_do_not(self):
        d={'timestamp':np.array([1000000,2000000]),'timestamp_sample':np.array([0,0])}
        np.testing.assert_allclose(a.time_data(d),[1,2])
        d['timestamp_sample'][1]=1500000
        np.testing.assert_allclose(a.time_data(d),[0,1.5])
    def test_phase_unwrap_recovers_after_initial_nan_and_does_not_bridge_gap(self):
        phase=a.unwrap_finite_phase([np.nan,6.2,.1,np.nan,.2,.4])
        self.assertTrue(np.isnan(phase[0]) and np.isnan(phase[3]))
        self.assertAlmostEqual(phase[2],2*np.pi+.1)
        np.testing.assert_allclose(phase[4:],[.2,.4])
    def test_timestamp_zero_not_supported(self):
        np.testing.assert_allclose(a.intervals([0,1,2]),[[1,2]])

if __name__=='__main__':unittest.main()
