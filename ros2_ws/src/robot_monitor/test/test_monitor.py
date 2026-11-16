import csv
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen, Request

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from monitor_core import Monitor, Recorder, Settings, clean, wheel_targets
from robot_monitor import make_server


def command(v=0.2, stage=4):
    return {'stage_id': stage, 'velocity': {'linear': {'y': v}, 'angular': {'z': 0.0}}}


def feedback(speeds=None, valid=True, stamp=None):
    m = {'valid': valid, 'speed_mps': speeds or [0.0]*4, 'yaw_rad': 0.0}
    if stamp is not None:
        m['header'] = {'stamp': {'sec': 1, 'nanosec': stamp}}
    return m


class StartupTests(unittest.TestCase):
    def ready(self, cfg=None):
        m = Monitor(cfg)
        for t in (0., .05, .10, .15, .20, .25):
            m.ingest('/base/wheel_state', feedback(), now=t)
        return m

    def begin(self, m):
        m.ingest('/base/route_command', command(), now=.30)

    def sample(self, m, t, speed, **kwargs):
        m.ingest('/base/route_command', command(**kwargs), now=t)
        m.ingest('/base/wheel_state', feedback(speed), now=t)

    def test_left_rear_delayed_and_mirror_does_not_replace_route(self):
        m = self.ready()
        self.begin(m)
        m.ingest('/base/cmd_vel', {'linear': {'y': -.4}}, now=.31)
        for t in (.35, .40, .45, .50):
            self.sample(m, t, [0, -.2, .2, -.2])
        for t in (.55, .60):
            self.sample(m, t, [.2, -.2, .2, -.2])
        e = m.events[0]
        self.assertEqual(e['status'], 'delayed')
        self.assertEqual(e['slowest'], 'A 左后')
        self.assertAlmostEqual(e['spread_ms'], 200.)
        self.assertEqual(m.command_topic, '/base/route_command')

    def test_equal_samples_are_within_resolution_not_exact_sync(self):
        m = self.ready()
        self.begin(m)
        self.sample(m, .35, [.2, -.2, .2, -.2])
        self.sample(m, .40, [.2, -.2, .2, -.2])
        self.assertEqual(m.events[0]['status'], 'within_resolution')
        self.assertEqual(m.events[0]['spread_ms'], 0)

    def test_starting_monitor_in_motion_is_insufficient(self):
        m = Monitor()
        self.begin(m)
        self.assertEqual(m.events[0]['status'], 'insufficient')

    def test_invalid_and_nan_feedback_never_prove_sync(self):
        for bad in (feedback(valid=False), feedback([float('nan'), 0, 0, 0])):
            m = self.ready()
            self.begin(m)
            m.ingest('/base/wheel_state', bad, now=.35)
            self.assertEqual(m.events[0]['status'], 'insufficient')

    def test_feedback_gap_and_stale_command(self):
        m = self.ready()
        self.begin(m)
        self.sample(m, .35, [.2, -.2, .2, -.2])
        m.ingest('/base/route_command', command(), now=.70)
        self.assertEqual(m.events[0]['status'], 'insufficient')
        m = self.ready(Settings(command_timeout_s=.10))
        self.begin(m)
        m.ingest('/base/wheel_state', feedback(), now=.45)
        self.assertEqual(m.events[0]['status'], 'insufficient')

    def test_stage_change_interrupts_without_rearming_moving_robot(self):
        m = self.ready()
        self.begin(m)
        m.ingest('/base/route_command', command(stage=6), now=.35)
        self.assertEqual(m.events[0]['status'], 'interrupted')
        self.assertIsNone(m.active)

    def test_duplicate_stamp_and_spike_do_not_confirm_start(self):
        m = self.ready()
        self.begin(m)
        speed = [.2, -.2, .2, -.2]
        m.ingest('/base/wheel_state', feedback(speed, stamp=1), now=.35)
        m.ingest('/base/wheel_state', feedback(speed, stamp=1), now=.36)
        self.assertIsNotNone(m.active)
        m.ingest('/base/wheel_state', feedback(stamp=2), now=.40)
        m.ingest('/base/wheel_state', feedback(speed, stamp=3), now=.45)
        self.assertIsNotNone(m.active)
        m.ingest('/base/wheel_state', feedback(speed, stamp=4), now=.50)
        self.assertEqual(m.events[0]['delays_ms'], [150.]*4)

    def test_wrong_direction_and_stalled_wheel(self):
        m = self.ready()
        self.begin(m)
        for i in range(1, 34):
            self.sample(m, .30+i*.05, [-.2, -.2, .2, -.2])
        self.assertEqual(m.events[0]['status'], 'not_started')
        self.assertIsNone(m.events[0]['delays_ms'][0])

    def test_low_target_is_not_called_stalled(self):
        m = self.ready()
        for i in range(34):
            self.sample(m, .30+i*.05, [0, 0, 0, 0], v=.01)
        self.assertEqual(m.events[0]['status'], 'insufficient')

    def test_ramp_start_and_second_run(self):
        m = self.ready()
        self.sample(m, .30, [0, 0, 0, 0], v=.005)
        self.sample(m, .35, [.1, -.1, .1, -.1], v=.1)
        self.sample(m, .40, [.2, -.2, .2, -.2])
        self.assertEqual(m.events[0]['status'], 'within_resolution')
        for t in (.45, .5, .55, .6, .65, .7):
            self.sample(m, t, [0, 0, 0, 0], v=0.)
        self.sample(m, .75, [0, 0, 0, 0])
        self.sample(m, .80, [.2, -.2, .2, -.2])
        self.sample(m, .85, [.2, -.2, .2, -.2])
        self.assertEqual(len(m.events), 2)
        self.assertEqual(m.events[0]['id'], 2)

    def test_kinematics_clamp_and_nonfinite(self):
        cfg = Settings(wheel_base_m=.22, track_width_m=.24)
        self.assertEqual(wheel_targets({'linear': {'y': .2}}, cfg), [.2, -.2, .2, -.2])
        v = wheel_targets({'angular': {'z': 1}}, cfg)
        self.assertAlmostEqual(v[0], -.23)
        self.assertAlmostEqual(v[2], .23)
        self.assertTrue(all(abs(x)<=1 for x in wheel_targets({'linear': {'x': 5, 'y': 5}}, cfg)))
        with self.assertRaises(ValueError):
            wheel_targets({'linear': {'x': float('nan')}}, cfg)
        self.assertEqual(clean({'bad': float('inf')}), {'bad': None})

    def test_concurrent_snapshot_and_callback(self):
        m = Monitor()
        errors = []
        def write():
            try:
                for i in range(400):
                    m.ingest('/custom/topic', {'data': i})
            except Exception as exc:
                errors.append(exc)
        t = threading.Thread(target=write)
        t.start()
        for _ in range(60):
            json.dumps(m.snapshot(), allow_nan=False)
        t.join()
        self.assertFalse(errors)


class RecordingAndHTTPTests(unittest.TestCase):
    def test_recorder_rotation_and_event_csv(self):
        with tempfile.TemporaryDirectory() as temp:
            r = Recorder(temp, max_mb=.001, keep_files=2)
            m = StartupTests().ready()
            m.recorder = r
            m.ingest('/base/route_command', command(), now=.30)
            for t in (.35, .40):
                m.ingest('/base/wheel_state', feedback([.2,-.2,.2,-.2]), now=t)
            for i in range(15):
                r.put({'kind': 'message', 'data': 'x'*500, 'index': i})
            r.close()
            self.assertFalse(r.error)
            self.assertEqual(len(list(r.directory.glob('telemetry*.jsonl'))), 2)
            for file in r.directory.glob('telemetry*.jsonl'):
                for line in file.read_text(encoding='utf-8').splitlines():
                    json.loads(line)
            with (r.directory/'startup_events.csv').open(encoding='utf-8-sig') as f:
                rows = list(csv.reader(f))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1][2], 'within_resolution')

    def test_http_assets_state_no_motion_endpoint(self):
        server = make_server(Monitor(mode='demo'), '127.0.0.1', 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = 'http://127.0.0.1:%d' % server.server_port
        try:
            for path in ('/', '/app.js', '/style.css', '/api/state'):
                with urlopen(url+path, timeout=3) as response:
                    self.assertEqual(response.status, 200)
                    self.assertTrue(response.read())
                    self.assertIn("default-src 'self'", response.headers['Content-Security-Policy'])
            with urlopen(url+'/api/state?compact=1', timeout=3) as response:
                s = json.load(response)
                self.assertEqual(s['mode'], 'demo')
                self.assertNotIn('history', s)
            for req in (url+'/../../package.xml', Request(url+'/drive', data=b'{}', method='POST')):
                with self.assertRaises(HTTPError):
                    urlopen(req, timeout=3)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
