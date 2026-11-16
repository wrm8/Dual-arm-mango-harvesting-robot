"""Write a single configured STM32 trigger frame; never read or retry commands."""
import os


# Strike protocol v1: AA 55 | command=01 | arm mask | 0D 0A (binary, no checksum).
# Mask bit 0 = left, bit 1 = right. Each complete frame is one strike event.
STRIKE_BAUD_RATE = 115200
STRIKE_COMMAND_HEX = {
    'left': 'AA 55 01 01 0D 0A',
    'right': 'AA 55 01 02 0D 0A',
    'both': 'AA 55 01 03 0D 0A',
}


def parse_frame(value):
    """Hex bytes, not a Python escape string: '01' differs from ASCII '31 0A'."""
    if not isinstance(value, str):
        raise ValueError('trigger frames must be quoted hexadecimal strings')
    try:
        frame = bytes.fromhex(value)
    except ValueError as error:
        raise ValueError('invalid trigger hex frame; use e.g. "01" or "31 0A"') from error
    if not 1 <= len(frame) <= 256:
        raise ValueError('each trigger frame must contain 1..256 bytes; configure the MCU protocol first')
    return frame


class SerialStrikerIO:
    def __init__(self, device, baud_rate, frames, chassis_device='/dev/stm32_chassis',
                 write_timeout_s=0.5):
        if not device or not chassis_device:
            raise ValueError('serial_device and chassis_serial_device must be nonempty')
        if os.path.realpath(device) == os.path.realpath(chassis_device):
            raise ValueError('striker and chassis must use different serial devices')
        if baud_rate <= 0 or not 0 < write_timeout_s <= 5:
            raise ValueError('invalid baud rate or serial write timeout')
        self.frames = {key: parse_frame(frames[key]) for key in ('left', 'right', 'both')}
        # Import only when physical serial control is enabled. No Jetson.GPIO dependency.
        import serial
        self.port = serial.Serial(
            port=None, baudrate=baud_rate, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=0, write_timeout=write_timeout_s, exclusive=True,
            xonxoff=False, rtscts=False, dsrdtr=False)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = device
        self.port.open()

    def trigger(self, arms):
        selected = set(arms)
        if not selected or len(selected) != len(arms) or not selected <= {'left', 'right'}:
            raise ValueError('arms must select left, right, or both without duplicates')
        key = 'both' if len(selected) == 2 else next(iter(selected))
        frame = self.frames[key]
        written = self.port.write(frame)
        # Do not retry a partial write: an earlier byte may already have triggered the MCU.
        if written != len(frame):
            raise IOError(f'incomplete trigger write ({written}/{len(frame)} bytes); not retried')
        # No read(), readline(), ACK wait, or automatic retransmission. Avoid unbounded tcdrain().
        return frame

    def close(self):
        self.port.close()
