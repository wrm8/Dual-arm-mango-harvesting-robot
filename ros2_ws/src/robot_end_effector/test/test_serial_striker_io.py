"""Test actual one-way serial bytes using a virtual tty, never motor hardware."""
import os
from pathlib import Path
import pty
import select

import pytest
import serial
import yaml

from robot_end_effector.serial_striker_io import (
    SerialStrikerIO, STRIKE_BAUD_RATE, STRIKE_COMMAND_HEX, parse_frame)


def configured_frames():
    document = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / 'config/serial_striker.yaml').read_text())
    return document['serial_striker_node']['ros__parameters']


def test_node_defaults_and_yaml_follow_the_same_strike_v1_contract():
    params = configured_frames()
    assert params['baud_rate'] == STRIKE_BAUD_RATE == 115200
    assert {key: params[key + '_command_hex'] for key in ('left', 'right', 'both')} == {
        'left': 'AA 55 01 01 0D 0A',
        'right': 'AA 55 01 02 0D 0A',
        'both': 'AA 55 01 03 0D 0A',
    } == STRIKE_COMMAND_HEX


@pytest.fixture
def wire():
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    params = configured_frames()
    io = SerialStrikerIO(port, params['baud_rate'], {
        key: params[key + '_command_hex'] for key in ('left', 'right', 'both')})
    try:
        yield io, master, port
    finally:
        io.close()
        os.close(master)
        os.close(slave)


def test_binary_and_ascii_protocols_are_distinct():
    assert parse_frame('01') == b'\x01'
    assert parse_frame('31 0A') == b'1\n'
    for invalid in ('', '1', '0x01', 'GG', '01 ' * 257, 1):
        with pytest.raises(ValueError):
            parse_frame(invalid)


def test_each_trigger_is_one_exact_frame_without_startup_bytes_or_feedback(wire):
    io, master, _ = wire
    assert not select.select([master], [], [], .02)[0]
    for arms, expected in ((['left'], b'\xAA\x55\x01\x01\x0D\x0A'),
                           (['right'], b'\xAA\x55\x01\x02\x0D\x0A'),
                           (['left', 'right'], b'\xAA\x55\x01\x03\x0D\x0A'),
                           (['right', 'left'], b'\xAA\x55\x01\x03\x0D\x0A'),
                           (['left'], b'\xAA\x55\x01\x01\x0D\x0A')):
        assert io.trigger(arms) == expected
        assert select.select([master], [], [], .5)[0]
        assert os.read(master, 256) == expected
        assert not select.select([master], [], [], .02)[0]


def test_rejects_chassis_port_even_through_another_symlink(wire, tmp_path):
    _, _, port = wire
    striker, chassis = tmp_path / 'striker', tmp_path / 'chassis'
    striker.symlink_to(port)
    chassis.symlink_to(port)
    with pytest.raises(ValueError, match='different serial'):
        SerialStrikerIO(str(striker), 115200, {}, str(chassis))


def test_second_serial_owner_is_rejected(wire):
    _, _, port = wire
    with pytest.raises(serial.SerialException):
        SerialStrikerIO(port, STRIKE_BAUD_RATE, STRIKE_COMMAND_HEX)


def test_partial_write_is_not_retried():
    class PartialPort:
        calls = 0

        def write(self, _frame):
            self.calls += 1
            return 1

    io = SerialStrikerIO.__new__(SerialStrikerIO)
    io.frames = {'left': b'1\n'}
    io.port = PartialPort()
    with pytest.raises(IOError, match='not retried'):
        io.trigger(['left'])
    assert io.port.calls == 1
