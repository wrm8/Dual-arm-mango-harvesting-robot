#!/usr/bin/env python3
"""Read-only ROS 2 telemetry dashboard. Also runs without ROS using --demo."""
import argparse
from dataclasses import asdict
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import threading
import time
from urllib.parse import urlparse, parse_qs

from monitor_core import DEFAULT_TOPICS, Monitor, Recorder, Settings


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=8765)
    p.add_argument('--demo', action='store_true', help='Synthetic data; no ROS or robot connection')
    p.add_argument('--log-dir', default='~/.ros/robot_monitor')
    p.add_argument('--no-record', action='store_true')
    p.add_argument('--max-log-mb', type=float, default=128.0)
    p.add_argument('--keep-log-files', type=int, default=4)
    p.add_argument('--chassis-config', default='', help='Read base_serial_node geometry from this YAML')
    p.add_argument('--command-source', choices=['auto', 'route', 'cmd_vel'], default='auto')
    p.add_argument('--start-threshold', type=float, default=0.03, help='Wheel startup threshold in m/s')
    p.add_argument('--delay-warning-ms', type=float, default=100.0)
    p.add_argument('--topic', action='append', default=[], help='Additional exact ROS topic to inspect/record')
    return p


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True


def make_server(monitor, host, port):
    root = Path(__file__).resolve().parent.parent / 'web'
    if not root.is_dir():
        root = Path(__file__).resolve().parent / 'web'
    assets = {'/': ('index.html', 'text/html; charset=utf-8'),
              '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
              '/style.css': ('style.css', 'text/css; charset=utf-8')}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            try:
                parsed = urlparse(self.path)
                if parsed.path == '/api/state':
                    state = monitor.snapshot()
                    if parse_qs(parsed.query).get('compact') == ['1']:
                        state.pop('history', None)
                    data = json.dumps(state, ensure_ascii=False, allow_nan=False).encode('utf-8')
                    mime = 'application/json; charset=utf-8'
                elif parsed.path in assets:
                    name, mime = assets[parsed.path]
                    data = (root / name).read_bytes()
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return DashboardServer((host, port), Handler)


def load_config(monitor, filename):
    import yaml
    with Path(filename).expanduser().open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    values = data['base_serial_node']['ros__parameters']
    monitor.set_geometry(values, str(Path(filename).expanduser().resolve()))
    for name in ('feedback_timeout_s', 'command_timeout_s'):
        if name in values:
            settings = asdict(monitor.settings)
            settings[name] = float(values[name])
            monitor.settings = Settings(**settings)
    monitor.geometry_locked = True


def run_ros(monitor, arguments, extra_topics, stop):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rcl_interfaces.srv import GetParameters

    class MonitorNode(Node):
        def __init__(self):
            super().__init__('robot_monitor')
            self.subscriptions_by_topic = {}
            self.wanted = set(DEFAULT_TOPICS + extra_topics)
            self.parameter_client = self.create_client(GetParameters, '/base_serial_node/get_parameters')
            self.parameter_future = None
            self.request_time = 0.0
            self.geometry_done = monitor.geometry_locked
            self.timer = self.create_timer(2.0, self.discover)
            self.discover()

        def received(self, topic, type_name, msg):
            try:
                monitor.ingest(topic, message_to_ordereddict(msg), type_name=type_name)
                monitor.fault(topic, '')
            except Exception as exc:
                monitor.fault(topic, exc)

        def discover(self):
            try:
                graph = self.get_topic_names_and_types()
                nodes = sorted(ns.rstrip('/')+'/'+name for name, ns in self.get_node_names_and_namespaces())
                with monitor.lock:
                    monitor.graph = [{'name': name, 'types': types} for name, types in sorted(graph)]
                    monitor.nodes = nodes
                qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                for topic, types in graph:
                    if topic not in self.wanted or topic in self.subscriptions_by_topic:
                        continue
                    if len(types) != 1:
                        monitor.fault(topic, '同一话题存在多个类型，请检查 ROS 环境')
                        continue
                    try:
                        cls = get_message(types[0])
                        sub = self.create_subscription(cls, topic, partial(self.received, topic, types[0]), qos)
                        self.subscriptions_by_topic[topic] = sub
                        self.get_logger().info('Watching %s [%s]' % (topic, types[0]))
                        monitor.fault(topic, '')
                    except Exception as exc:
                        monitor.fault(topic, '接口加载失败：%s；请 source 正确的 install/setup.bash' % exc)
                self.update_geometry()
                monitor.fault('discovery', '')
            except Exception as exc:
                monitor.fault('discovery', exc)

        def update_geometry(self):
            if self.geometry_done:
                return
            if self.parameter_future is not None:
                if self.parameter_future.done():
                    result = self.parameter_future.result()
                    names = ['wheel_base_m', 'track_width_m', 'max_wheel_speed_mps',
                             'feedback_timeout_s', 'command_timeout_s']
                    values = {}
                    if result:
                        for name, v in zip(names, result.values):
                            if v.type == 3:
                                values[name] = v.double_value
                            elif v.type == 2:
                                values[name] = v.integer_value
                    try:
                        if monitor.set_geometry(values, '/base_serial_node 实时参数'):
                            with monitor.lock:
                                cfg = asdict(monitor.settings)
                                cfg.update({k: values[k] for k in ('feedback_timeout_s', 'command_timeout_s') if k in values})
                                monitor.settings = Settings(**cfg)
                            self.geometry_done = True
                            monitor.fault('geometry', '')
                    except Exception as exc:
                        monitor.fault('geometry', exc)
                    self.parameter_future = None
                elif time.monotonic()-self.request_time > 5:
                    self.parameter_future.cancel()
                    self.parameter_future = None
                return
            if self.parameter_client.service_is_ready():
                request = GetParameters.Request()
                request.names = ['wheel_base_m', 'track_width_m', 'max_wheel_speed_mps',
                                 'feedback_timeout_s', 'command_timeout_s']
                self.parameter_future = self.parameter_client.call_async(request)
                self.request_time = time.monotonic()

    rclpy.init(args=arguments)
    node = None
    try:
        node = MonitorNode()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def demo_feed(monitor, stop):
    start = time.monotonic()
    while not stop.wait(0.05):
        now = time.monotonic()
        t = (now-start) % 6.0
        moving = 1.0 <= t < 4.0
        twist = {'linear': {'x': 0.0, 'y': 0.22 if moving else 0.0, 'z': 0.0},
                 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        monitor.ingest('/base/route_command', {'stage_id': 4, 'velocity': twist}, now)
        speeds = [0.22 if moving and t > 1.25 else 0.0,
                  -0.22 if moving else 0.0, 0.22 if moving else 0.0, -0.22 if moving else 0.0]
        yaw = 0.035*math.sin((t-1)*3)*math.exp(-(t-1)) if moving else 0.0
        monitor.ingest('/base/wheel_state', {'valid': True, 'speed_mps': speeds,
                        'encoder_delta': [round(v*330) for v in speeds], 'yaw_rad': yaw}, now)
        monitor.ingest('/base/state', {'vx_mps': 0.0, 'vy_mps': sum(abs(v) for v in speeds)/4,
                        'wz_rad_s': 0.0, 'yaw_rad': yaw, 'stopped': not moving, 'command_timed_out': False}, now)
        monitor.ingest('/base/ultrasonic', {'front_m': 0.20, 'rear_m': 0.21, 'left_m': 0.4,
                        'right_m': 0.6, 'front_valid': True, 'rear_valid': True,
                        'left_valid': True, 'right_valid': True}, now)


def main(argv=None):
    args, ros_args = parser().parse_known_args(argv)
    # Reject misspelled monitor options, but allow the normal ros2 run/launch --ros-args block.
    if ros_args and ros_args[0] != '--ros-args':
        parser().error('unrecognized arguments: ' + ' '.join(ros_args))
    cfg = Settings(start_threshold=args.start_threshold, delay_warning_ms=args.delay_warning_ms,
                   command_source=args.command_source)
    monitor = Monitor(cfg, mode='demo' if args.demo else 'live')
    if args.chassis_config:
        load_config(monitor, args.chassis_config)
    server = make_server(monitor, args.host, args.port)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    if not args.no_record:
        monitor.recorder = Recorder(args.log_dir, args.max_log_mb, args.keep_log_files)
        monitor.record({'kind': 'metadata', 'mode': monitor.mode, 'settings': asdict(monitor.settings),
                        'wheel_order': ['left_rear', 'left_front', 'right_front', 'right_rear']})
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print('Robot monitor: http://%s:%d  MODE=%s  READ-ONLY' % (args.host, server.server_port, monitor.mode), flush=True)
    if monitor.recorder:
        print('Recording: ' + str(monitor.recorder.directory), flush=True)
    try:
        if args.demo:
            demo_feed(monitor, stop)
        else:
            run_ros(monitor, ros_args, args.topic, stop)
    except ImportError as exc:
        raise SystemExit('ROS dependency unavailable: %s. Source /opt/ros/humble/setup.bash and install/setup.bash, or use --demo.' % exc)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        if monitor.recorder:
            monitor.recorder.close()


if __name__ == '__main__':
    main()
