"""ROS-independent telemetry store and conservative wheel startup estimates."""
import copy
import csv
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import threading
import time


WHEELS = ['A 左后', 'B 左前', 'C 右前', 'D 右后']
DEFAULT_TOPICS = [
    '/base/wheel_state', '/base/state', '/base/cmd_vel', '/base/route_command',
    '/base/stage_feedback', '/base/ultrasonic', '/base/odom', '/base/imu',
    '/base/vision_stop_state', '/vision/row_observation', '/vision/observations',
    '/vision/harvest_targets', '/vision/mango_target', '/joint_states', '/rosout',
]


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


@dataclass
class Settings:
    wheel_base_m: float = 0.220
    track_width_m: float = 0.240
    max_wheel_speed_mps: float = 1.0
    feedback_timeout_s: float = 0.30
    command_timeout_s: float = 0.35
    start_threshold: float = 0.03
    rest_threshold: float = 0.015
    rest_duration_s: float = 0.20
    confirm_samples: int = 2
    startup_window_s: float = 1.50
    delay_warning_ms: float = 100.0
    command_source: str = 'auto'

    def __post_init__(self):
        for key, value in asdict(self).items():
            if key != 'command_source' and (not math.isfinite(value) or value <= 0):
                raise ValueError(key + ' must be finite and positive')
        if self.command_source not in ('auto', 'route', 'cmd_vel'):
            raise ValueError('command_source must be auto, route or cmd_vel')
        if self.rest_threshold >= self.start_threshold:
            raise ValueError('rest_threshold must be less than start_threshold')


def wheel_targets(twist, settings):
    vx = float(twist.get('linear', {}).get('x', 0.0))
    vy = float(twist.get('linear', {}).get('y', 0.0))
    wz = float(twist.get('angular', {}).get('z', 0.0))
    if not all(math.isfinite(x) for x in (vx, vy, wz)):
        raise ValueError('non-finite velocity')
    vx, vy, wz = max(-1., min(1., vx)), max(-1., min(1., vy)), max(-2., min(2., wz))
    r = (settings.wheel_base_m + settings.track_width_m) / 2.0
    limit = settings.max_wheel_speed_mps
    return [max(-limit, min(limit, x)) for x in
            (vx + vy - r*wz, vx - vy - r*wz, vx + vy + r*wz, vx - vy + r*wz)]


class Recorder:
    """Bounded background writer: a slow disk never blocks a ROS callback."""
    def __init__(self, root, max_mb=128, keep_files=4):
        if max_mb <= 0 or keep_files < 1:
            raise ValueError('invalid recording limits')
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        self.directory = Path(root).expanduser().resolve() / stamp
        self.directory.mkdir(parents=True, exist_ok=False)
        self.max_bytes = int(max_mb * 1024 * 1024)
        self.keep_files = keep_files
        self.pending = queue.Queue(maxsize=4096)
        self.dropped = 0
        self.error = ''
        self.closed = False
        self.thread = threading.Thread(target=self._worker, name='telemetry-writer', daemon=True)
        self.thread.start()

    def put(self, item):
        if self.closed:
            return
        try:
            self.pending.put_nowait(clean(item))
        except queue.Full:
            self.dropped += 1

    def _worker(self):
        raw = None
        try:
            raw = (self.directory / 'telemetry.jsonl').open('a', encoding='utf-8')
            with (self.directory / 'startup_events.csv').open('w', newline='', encoding='utf-8-sig') as events:
                writer = csv.writer(events)
                writer.writerow(['id', 'stage_id', 'status', 'reason', 'A_ms', 'B_ms', 'C_ms',
                                 'D_ms', 'spread_ms', 'slowest', 'feedback_interval_ms', 'yaw_change_deg'])
                size, count, last_flush = 0, 0, time.monotonic()
                while True:
                    try:
                        item = self.pending.get(timeout=0.5)
                    except queue.Empty:
                        item = 'flush'
                    if item is None:
                        break
                    if item != 'flush':
                        line = json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n'
                        encoded_size = len(line.encode('utf-8'))
                        if size + encoded_size > self.max_bytes:
                            raw.close()
                            for index in range(self.keep_files - 1, 0, -1):
                                src = self.directory / ('telemetry.jsonl' if index == 1 else 'telemetry.%d.jsonl' % (index-1))
                                dst = self.directory / ('telemetry.%d.jsonl' % index)
                                if src.exists():
                                    src.replace(dst)
                            raw = (self.directory / 'telemetry.jsonl').open('w', encoding='utf-8')
                            size = 0
                        raw.write(line)
                        size += encoded_size
                        if item.get('kind') == 'startup':
                            e = item['event']
                            writer.writerow([e.get(k, '') for k in ('id', 'stage_id', 'status', 'reason')] +
                                            e['delays_ms'] + [e.get(k, '') for k in
                                            ('spread_ms', 'slowest', 'feedback_interval_ms', 'yaw_change_deg')])
                        count += 1
                    if time.monotonic() - last_flush > 1.0 or count >= 100:
                        raw.flush()
                        events.flush()
                        count, last_flush = 0, time.monotonic()
                raw.flush()
        except Exception as exc:
            self.error = str(exc)
        finally:
            if raw is not None:
                raw.close()

    def close(self):
        self.closed = True
        if self.thread.is_alive():
            while self.thread.is_alive():
                try:
                    self.pending.put(None, timeout=0.2)
                    break
                except queue.Full:
                    pass
            self.thread.join(timeout=5)


class Monitor:
    def __init__(self, settings=None, recorder=None, mode='live'):
        self.settings = settings or Settings()
        self.recorder = recorder
        self.mode = mode
        self.lock = threading.RLock()
        self.started = time.monotonic()
        self.topics = {}
        self.graph = []
        self.nodes = []
        self.history = deque(maxlen=1200)
        self.events = deque(maxlen=100)
        self.logs = deque(maxlen=60)
        self.active = None
        self.event_id = 0
        self.last_feedback = None
        self.feedback_stamp = None
        self.intervals = deque(maxlen=80)
        self.rest_since = None
        self.last_speed = [0.0]*4
        self.targets = [0.0]*4
        self.command_time = None
        self.command_topic = None
        self.route_seen = False
        self.stage = None
        self.geometry_source = '默认尺寸，请核对或指定 --chassis-config'
        self.geometry_locked = False
        self.faults = {}

    def record(self, item):
        if self.recorder:
            self.recorder.put(item)

    def fault(self, key, error):
        with self.lock:
            if error:
                self.faults[key] = str(error)
            else:
                self.faults.pop(key, None)

    def set_geometry(self, values, source):
        with self.lock:
            if self.active:
                return False
            new = asdict(self.settings)
            for key in ('wheel_base_m', 'track_width_m', 'max_wheel_speed_mps'):
                if key not in values:
                    raise ValueError('missing geometry parameter: ' + key)
                new[key] = float(values[key])
            self.settings = Settings(**new)
            self.geometry_source = source
            self.record({'kind': 'settings', 'settings': new, 'geometry_source': source})
            return True

    def ingest(self, topic, message, now=None, type_name=''):
        now = time.monotonic() if now is None else now
        with self.lock:
            self._tick(now)
            entry = self.topics.setdefault(topic, {'times': deque(maxlen=120), 'count': 0})
            entry['times'].append(now)
            entry.update(last=now, message=clean(message), type=type_name, count=entry['count']+1)
            self.record({'kind': 'message', 'time': now, 'wall_time': time.time(),
                         'topic': topic, 'type': type_name, 'message': message})
            if topic in ('/base/route_command', '/base/cmd_vel'):
                self._command(topic, message, now)
            elif topic == '/base/wheel_state':
                self._feedback(message, now)
            elif topic == '/rosout' and int(message.get('level', 0)) >= 30:
                self.logs.appendleft({'time': now, **clean(message)})

    def _command(self, topic, message, now):
        source = self.settings.command_source
        if topic.endswith('route_command'):
            if source == 'cmd_vel':
                return
            self.route_seen = True
            twist = message.get('velocity', {})
            stage = message.get('stage_id')
        else:
            if source == 'route' or (source == 'auto' and self.route_seen):
                return
            twist, stage = message, None
        try:
            targets = wheel_targets(twist, self.settings)
        except (TypeError, ValueError) as exc:
            self.fault('command', exc)
            if self.active:
                self._finish('insufficient', '收到非有限或损坏的速度指令', now)
            return
        self.fault('command', '')
        old_moving = any(abs(v) > 1e-6 for v in self.targets)
        moving = any(abs(v) > 1e-6 for v in targets)
        if self.active:
            if stage != self.stage or topic != self.command_topic:
                self._finish('interrupted', '路线段或指令来源改变', now)
            elif not moving:
                self._finish('interrupted', '起步确认前已收到停车指令', now)
            elif any(a*b < 0 for a, b in zip(targets, self.active['targets'])):
                self._finish('interrupted', '轮子目标方向改变', now)
            if self.active:
                for i, value in enumerate(targets):
                    self.active['peak_target'][i] = max(self.active['peak_target'][i], abs(value))
                    if abs(self.active['targets'][i]) <= 1e-6:
                        self.active['targets'][i] = value
        self.targets, self.command_time, self.command_topic, self.stage = targets, now, topic, stage
        if moving and not old_moving:
            self.event_id += 1
            rest_ready = (self.last_feedback is not None and now - self.last_feedback <= self.settings.feedback_timeout_s
                          and self.rest_since is not None and now - self.rest_since >= self.settings.rest_duration_s)
            yaw = self.topics.get('/base/wheel_state', {}).get('message', {}).get('yaw_rad')
            self.active = {'id': self.event_id, 'stage_id': stage, 'start_time': now,
                           'targets': list(targets), 'peak_target': [abs(v) for v in targets], 'delays_ms': [None]*4,
                           'first_seen': [None]*4, 'streak': [0]*4, 'yaw_start': yaw,
                           'yaw_change_deg': None, 'status': 'collecting', 'reason': '正在观察四轮起转'}
            if not rest_ready:
                self._finish('insufficient', '起步前没有连续静止的有效反馈；请提前打开监测', now)
            self.rest_since = None

    def _feedback(self, message, now):
        speeds = message.get('speed_mps', [])
        if (not message.get('valid', False) or len(speeds) != 4 or
                not all(isinstance(v, (int, float)) and math.isfinite(v) for v in speeds)):
            self.rest_since = None
            if self.active:
                self._finish('insufficient', '轮速反馈无效', now)
            return
        stamp = message.get('header', {}).get('stamp')
        if stamp and stamp == self.feedback_stamp:
            return
        self.feedback_stamp = copy.deepcopy(stamp)
        if self.last_feedback is not None:
            gap = now - self.last_feedback
            if gap > self.settings.feedback_timeout_s:
                self.rest_since = None
                if self.active:
                    self._finish('insufficient', '轮速反馈中断，不能判定同步性', now)
            elif gap > 0:
                self.intervals.append(gap)
        self.last_feedback, self.last_speed = now, list(speeds)
        if all(abs(v) < self.settings.rest_threshold for v in speeds):
            if self.rest_since is None:
                self.rest_since = now
        else:
            self.rest_since = None
        yaw = message.get('yaw_rad')
        self.history.append({'t': now, 'speed': list(speeds), 'target': list(self.targets),
                             'yaw': yaw, 'stage': self.stage})
        if self.active:
            e = self.active
            if isinstance(yaw, (int, float)) and e['yaw_start'] is not None:
                e['yaw_change_deg'] = math.degrees(math.atan2(math.sin(yaw-e['yaw_start']), math.cos(yaw-e['yaw_start'])))
            for i in range(4):
                if e['delays_ms'][i] is not None:
                    continue
                if (abs(e['targets'][i]) > 1e-6 and
                        speeds[i] * (1 if e['targets'][i] > 0 else -1) >= self.settings.start_threshold):
                    if e['streak'][i] == 0:
                        e['first_seen'][i] = now
                    e['streak'][i] += 1
                    if e['streak'][i] >= self.settings.confirm_samples:
                        e['delays_ms'][i] = round((e['first_seen'][i]-e['start_time'])*1000, 1)
                else:
                    e['streak'][i], e['first_seen'][i] = 0, None
            if all(v is not None for v in e['delays_ms']):
                spread = max(e['delays_ms']) - min(e['delays_ms'])
                self._finish('delayed' if spread >= self.settings.delay_warning_ms else 'within_resolution',
                             '检测到起转时间差' if spread >= self.settings.delay_warning_ms else '未观察到超过阈值的起转差；不代表严格同步', now)

    def _finish(self, status, reason, now):
        e = self.active
        if not e:
            return
        e.update(status=status, reason=reason, end_time=now)
        times = e['delays_ms']
        e['spread_ms'] = round(max(times)-min(times), 1) if all(x is not None for x in times) else None
        e['slowest'] = WHEELS[times.index(max(times))] if all(x is not None for x in times) and max(times)>min(times) else ''
        e['feedback_interval_ms'] = round(sum(self.intervals)/len(self.intervals)*1000, 1) if self.intervals else None
        for key in ('first_seen', 'streak'):
            e.pop(key, None)
        self.events.appendleft(e)
        self.record({'kind': 'startup', 'event': e})
        self.active = None

    def _tick(self, now):
        if not self.active:
            return
        if self.last_feedback is None or now-self.last_feedback > self.settings.feedback_timeout_s:
            self._finish('insufficient', '轮速反馈超时', now)
        elif self.command_time is None or now-self.command_time > self.settings.command_timeout_s:
            self._finish('insufficient', '速度指令超时', now)
        elif now-self.active['start_time'] > self.settings.startup_window_s:
            if any(v < self.settings.start_threshold*1.5 for v in self.active['peak_target']):
                self._finish('insufficient', '部分轮子目标过小，不能公平比较四轮起转', now)
            else:
                self._finish('not_started', '观察窗口内有轮子未确认起转；检查编码器和驱动', now)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            self._tick(now)
            topics = {}
            for name in DEFAULT_TOPICS + [x for x in self.topics if x not in DEFAULT_TOPICS]:
                e = self.topics.get(name)
                times = e['times'] if e else []
                recent = [x for x in times if now-x <= 5.0]
                hz = (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else 0.0
                topics[name] = {'age_s': round(now-e['last'], 3) if e else None,
                                'hz': round(hz, 1), 'count': e['count'] if e else 0,
                                'type': e.get('type', '') if e else '', 'message': e.get('message') if e else None}
            return clean({'mode': self.mode, 'now': now, 'uptime_s': now-self.started,
                          'settings': asdict(self.settings), 'geometry_source': self.geometry_source,
                          'topics': topics, 'graph': self.graph, 'nodes': self.nodes,
                          'command': {'topic': self.command_topic, 'stage': self.stage, 'target': self.targets,
                                      'age_s': now-self.command_time if self.command_time is not None else None},
                          'history': list(self.history)[-400:], 'startup': self.active,
                          'events': list(self.events), 'logs': list(self.logs), 'faults': dict(self.faults),
                          'recording': {'directory': str(self.recorder.directory), 'dropped': self.recorder.dropped,
                                        'error': self.recorder.error} if self.recorder else None})
