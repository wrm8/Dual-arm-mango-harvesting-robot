#!/usr/bin/env python3
"""Safe first-contact test for one Hiwonder/LX-style serial bus servo.

The default operation only reads and prints the current position.  A motion command
requires --move and is a small relative delta from that measured position.
"""

import argparse
import sys

from sdk.hiwonder_servo_controller import HiwonderServoController


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="for example /dev/serial/by-id/...")
    parser.add_argument("--id", type=int, required=True, help="servo ID, 0 to 253")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--move", action="store_true", help="allow the small relative movement")
    parser.add_argument("--delta", type=int, default=10, help="relative tick move, default +10; allowed -20 to +20")
    parser.add_argument("--duration-ms", type=int, default=1000, help="motion time, 20 to 30000")
    args = parser.parse_args()

    if not 0 <= args.id <= 253:
        parser.error("--id must be 0 to 253")
    if not -20 <= args.delta <= 20:
        parser.error("--delta must be between -20 and +20 for a first test")
    if not 20 <= args.duration_ms <= 30000:
        parser.error("--duration-ms must be 20 to 30000")

    controller = HiwonderServoController(args.port, args.baud)
    try:
        current = controller.get_servo_position(args.id)
        if current is None:
            print("No valid position response. Check power, servo ID, port and TX/RX direction GPIO.", file=sys.stderr)
            return 2
        print(f"Servo {args.id}: current position = {current}")
        if not args.move:
            print("Read-only test complete. Add --move to command a relative motion of at most 20 ticks.")
            return 0
        target = max(0, min(1000, current + args.delta))
        print(f"Moving servo {args.id}: {current} -> {target} in {args.duration_ms} ms")
        controller.set_servo_position(args.id, target, args.duration_ms)
        return 0
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
