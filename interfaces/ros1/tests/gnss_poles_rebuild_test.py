#!/usr/bin/env python3
"""Exercise automatic rebuilding, repeat runs and rollback without a ROS master."""
import hashlib
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

EXECUTABLE = str(Path(sys.argv.pop(1)).resolve())


def read_pcd(path):
    with path.open('rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise AssertionError('Missing PCD data header')
            if line.startswith(b'DATA binary'):
                return np.frombuffer(stream.read(), dtype='<f4').reshape(-1, 3)


class RebuildTest(unittest.TestCase):
    def test_rebuild_repeat_and_failure_preserves_original(self):
        with tempfile.TemporaryDirectory(prefix='bievr-gnss-poles-') as temp:
            root = Path(temp)
            poles = root / 'poles'
            poles.mkdir()
            (poles / 'landmarks.txt').write_text('original pole results\n')
            (poles / 'rail_vector_map.txt').write_text('original rail results\n')
            settings = {'pole_extraction': {
                'enable': True, 'frames_per_keyframe': 10,
                'min_frames_per_keyframe': 10, 'min_partial_frames': 2,
                'keyframe_overlap_frames': 2, 'keyframe_distance_m': 0.,
                'max_queue': 1, 'cylinder_fit_enable': False,
                'min_points_per_cell': 8, 'min_height_span_m': 5.,
                'min_continuous_layers': 20, 'save_keyframe_clouds': True,
            }, 'rail_extraction': {'enable': True}}
            (poles / 'extraction_config.yaml').write_text(yaml.safe_dump(settings))
            stamps = np.arange(1000., 1060.)
            raw = np.column_stack((stamps, -np.arange(60.), np.zeros((60, 2)),
                                   np.zeros((60, 3)), np.ones(60)))
            np.savetxt(root / 'odometry_tum.txt', raw, fmt='%.9f')
            fixes = np.column_stack((stamps, 100. - np.arange(60.), np.full(60, 20.),
                                     np.full(60, 10.), np.full((60, 3), .01)))
            np.savetxt(root / 'gnss.txt', fixes, fmt='%.9f',
                       header='origin_lla 30 114 0')
            rails = []
            for x in np.arange(-10., 10.01, .1):
                for lateral_bin in [-15, 14]:
                    for offset in [0, -3, -2, 2, 3]:
                        rails.append([x, (lateral_bin + offset + .5) * .05,
                                      -4.5 if offset == 0 else -4.65, 1.])
            rails = np.asarray(rails)
            with (root / 'scans.bin').open('wb') as stream:
                stream.write(b'BIEVRS1\0')
                for index, stamp in enumerate(stamps):
                    pillar = np.array([[-15. + index + .2*np.cos(angle),
                                        4. + .2*np.sin(angle), z, 1.]
                                       for z in np.arange(-4., 6.01, .1)
                                       for angle in np.linspace(0, 2*np.pi, 20, endpoint=False)])
                    cloud = np.vstack((rails, pillar)).astype('<f4')
                    stream.write(struct.pack('<QQ', int(stamp * 1e9), len(cloud)))
                    stream.write(cloud.tobytes())
            command = [EXECUTABLE, '--gnss_enu', str(root / 'gnss.txt'),
                       '--lio_trajectory', str(root / 'odometry_tum.txt'),
                       '--scans', str(root / 'scans.bin'),
                       '--output_directory', str(root / 'optimized'), '--skip-output-map']
            environment = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:1', MPLBACKEND='Agg')

            def run(extra=(), success=True):
                result = subprocess.run(command + list(extra), env=environment,
                                        text=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, timeout=90)
                if success:
                    self.assertEqual(result.returncode, 0, result.stdout)
                else:
                    self.assertNotEqual(result.returncode, 0, result.stdout)
                return result.stdout

            run()
            trajectory = np.loadtxt(root / 'optimized/gnss_optimized_tum.txt')
            obj_lines = (root / 'optimized/trajectory.obj').read_text().splitlines()
            vertices = np.array([[float(v) for v in line.split()[1:]]
                                 for line in obj_lines if line.startswith('v ')])
            np.testing.assert_allclose(vertices, trajectory[:, 1:4], atol=1e-8)
            self.assertEqual(obj_lines[-1].split(),
                             ['l'] + [str(i + 1) for i in range(len(trajectory))])
            backups = list(root.glob('poles.before_gnss_*'))
            self.assertEqual(len(backups), 1)
            self.assertEqual((backups[0] / 'landmarks.txt').read_text(), 'original pole results\n')
            summary = yaml.safe_load((poles / 'summary.yaml').read_text())
            self.assertEqual(summary['received_frames'], 60)
            self.assertEqual(summary['dropped_keyframes'], 0)
            self.assertGreater(summary['landmarks'], 0)
            self.assertGreater(summary['rail_samples'], 100)
            points = read_pcd(poles / 'rails.pcd')
            self.assertGreater(len(points), 1000)
            self.assertGreater(np.ptp(points[:, 0]), 40.)
            self.assertTrue(np.all((np.abs(points[:, 2] - 5.5) < .02) |
                                   (np.abs(points[:, 2] - 5.35) < .02)))
            self.assertGreater(len(read_pcd(poles / 'poles.pcd')), 0)
            saved_config = yaml.safe_load((poles / 'extraction_config.yaml').read_text())
            self.assertEqual(saved_config['pole_extraction']['output_directory'], str(poles))
            self.assertEqual(yaml.safe_load((poles / 'gnss_optimization.yaml').read_text())['accepted_scans'], 60)
            run()  # Rebuild from body scans again, without applying ENU twice.
            np.testing.assert_allclose(read_pcd(poles / 'rails.pcd'), points, atol=1e-5)
            self.assertEqual(len(list(root.glob('poles.before_gnss_*'))), 2)
            hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in poles.iterdir()}
            (root / 'scans.bin').rename(root / 'scans.saved')
            self.assertIn('requires scan archive', run(success=False))
            self.assertEqual(hashes, {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in poles.iterdir()})
            run(['--skip-poles'])
            self.assertEqual(hashes, {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in poles.iterdir()})
            (root / 'scans.saved').rename(root / 'scans.bin')
            (root / 'bad.yaml').write_text('pole_extraction:\n  frames_per_keyframe: 0\n')
            run(['--pole_params_file', str(root / 'bad.yaml')], success=False)
            self.assertEqual(hashes, {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in poles.iterdir()})
            settings['pole_extraction']['min_height_span_m'] = 100.
            (root / 'empty.yaml').write_text(yaml.safe_dump(settings))
            self.assertIn('erase a nonempty', run(
                ['--pole_params_file', str(root / 'empty.yaml')], success=False))
            self.assertEqual(hashes, {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in poles.iterdir()})
            self.assertEqual(list(root.glob('poles.gnss_pending_*')), [])


if __name__ == '__main__':
    unittest.main()
