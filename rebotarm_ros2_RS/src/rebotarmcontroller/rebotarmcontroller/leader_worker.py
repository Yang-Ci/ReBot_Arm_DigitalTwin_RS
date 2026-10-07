"""Isolated UART worker: no ROS imports and no follower/CAN access."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

from rebotarmcontroller.leader_policy import fresh_angles


def emit(**value):
    print(json.dumps(value, allow_nan=False), flush=True)


def run(port, probe=False, zero=False, unlock=False):
    bus = None
    try:
        if port != "mock":
            from motorbridge_smart_servo import FashionStarServo
            bus = FashionStarServo(port, baudrate=1_000_000)
            ids = [i for i in range(7) if bus.ping(i)]
        else:
            ids = list(range(7))
        if probe:
            emit(event="probe", ids=ids)
            return
        if ids != list(range(7)):
            raise RuntimeError(f"missing leader servo IDs: {sorted(set(range(7)) - set(ids))}")
        if (unlock or zero) and bus:
            # Explicit confirmation only. Same origin/unlock sequence as Wiki.
            for i in ids:
                bus.unlock(i)
                time.sleep(0.01)
                if zero:
                    bus.set_origin_point(i)
                    bus.reset_multi_turn(i)
            time.sleep(0.1)
        emit(event="ready", ids=ids)
        seq = 0
        started = time.monotonic()
        while True:
            began = time.monotonic()
            try:
                angles = fresh_angles(bus.sync_monitor(ids)) if bus else [
                    8 * math.sin((began - started) * 0.4),
                    8 * math.sin((began - started) * 0.4),
                    -8 * math.sin((began - started) * 0.4), 0, 0, 0,
                    5 * (1 - math.cos((began - started) * 0.4))]
                seq += 1
                emit(event="sample", angles=angles, seq=seq, at=began)
            except Exception as exc:
                emit(event="error", message=str(exc))
            time.sleep(max(0, 1 / 30 - (time.monotonic() - began)))
    except Exception as exc:
        emit(event="fatal", message=str(exc))
    finally:
        if bus:
            bus.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--zero", action="store_true")
    parser.add_argument("--unlock", action="store_true")
    args = parser.parse_args()
    run(args.port, args.probe, args.zero, args.unlock)
