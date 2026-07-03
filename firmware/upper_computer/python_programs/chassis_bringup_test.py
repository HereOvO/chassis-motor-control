#!/usr/bin/env python3
"""USART1 bring-up helper for the migrated chassis firmware.

Supported firmware commands include velocity mode and raw PWM test mode:

    VEL,<vx_mps>,<vy_mps>,<wz_radps>
    PWMTEST,<motor_id>,<direction>,<duty>
    RAWPWM,<motor_id>,<direction>,<duty>

The firmware emits ASCII feedback lines at 10 Hz. This script can print those
lines while a test is running.

Motor index mapping:

    0 = left-front wheel
    1 = left-rear wheel
    2 = right-rear wheel
    3 = right-front wheel
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc


DEFAULT_BAUD = 115200
MOTOR_LABELS = {
    0: "left-front",
    1: "left-rear",
    2: "right-rear",
    3: "right-front",
}


@dataclass(frozen=True)
class VelocityStep:
    vx: float
    vy: float
    wz: float
    duration_s: float


def send_line(port: serial.Serial, line: str, delay_s: float = 0.05) -> None:
    command = line.strip()
    print(f">> {command}")
    port.write((command + "\r\n").encode("ascii"))
    port.flush()
    if delay_s > 0.0:
        time.sleep(delay_s)


def read_feedback(port: serial.Serial, duration_s: float) -> None:
    deadline = time.time() + max(0.0, duration_s)
    buffer = bytearray()
    while time.time() < deadline:
        chunk = port.read(512)
        if not chunk:
            continue
        buffer.extend(chunk)
        while b"\n" in buffer:
            line, _, rest = buffer.partition(b"\n")
            buffer = bytearray(rest)
            text = line.decode("ascii", errors="replace").strip()
            if text:
                print(f"<< {text}")
    if buffer:
        text = buffer.decode("ascii", errors="replace").strip()
        if text:
            print(f"<< {text}")


def stop_chassis(port: serial.Serial) -> None:
    send_line(port, "VEL,0.000,0.000,0.000")
    send_line(port, "ZERO")
    send_line(port, "ENABLE,0")


def send_commands(port: serial.Serial, commands: Iterable[str]) -> None:
    for command in commands:
        send_line(port, command)


def parse_set_args(values: list[str]) -> list[str]:
    commands: list[str] = []
    for item in values:
        if "=" not in item:
            raise ValueError(f"invalid --set item {item!r}, expected id=value")
        param_id, value = item.split("=", 1)
        commands.append(f"SET,{int(param_id, 10)},{float(value):.6g}")
    return commands


def build_steps(args: argparse.Namespace) -> list[VelocityStep]:
    if args.pattern == "forward":
        return [VelocityStep(args.vx, 0.0, 0.0, args.duration)]
    if args.pattern == "strafe":
        return [VelocityStep(0.0, args.vy, 0.0, args.duration)]
    if args.pattern == "rotate":
        return [VelocityStep(0.0, 0.0, args.wz, args.duration)]
    if args.pattern == "square":
        return [
            VelocityStep(args.vx, 0.0, 0.0, args.duration),
            VelocityStep(0.0, args.vy, 0.0, args.duration),
            VelocityStep(-args.vx, 0.0, 0.0, args.duration),
            VelocityStep(0.0, -args.vy, 0.0, args.duration),
        ]
    raise ValueError(f"unsupported pattern: {args.pattern}")


def run_velocity(port: serial.Serial, args: argparse.Namespace) -> None:
    send_commands(port, ("ASCII", "ZERO", "PROFILE,1", "MODE,1", "ENABLE,1"))
    for step in build_steps(args):
        send_line(port, f"VEL,{step.vx:.3f},{step.vy:.3f},{step.wz:.3f}")
        read_feedback(port, step.duration_s if args.monitor else step.duration_s)
    if args.no_stop:
        print("Leaving velocity command active because --no-stop was set.")
    else:
        stop_chassis(port)


def run_pwmtest(port: serial.Serial, args: argparse.Namespace) -> None:
    if args.motor is None or args.direction is None:
        raise ValueError("pwmtest requires --motor and --direction")
    if args.motor < 0 or args.motor > 3:
        raise ValueError("--motor must be 0..3")
    if args.direction not in (-1, 0, 1):
        raise ValueError("--direction must be -1, 0, or 1")
    if args.duty < 0.0 or args.duty > 1.0:
        raise ValueError("--duty must be 0.0..1.0")

    send_line(port, "ASCII")
    print(f"Testing motor {args.motor}: {MOTOR_LABELS[args.motor]}")
    send_line(port, f"PWMTEST,{args.motor},{args.direction},{args.duty:.3f}")
    read_feedback(port, args.duration if args.monitor else args.duration)
    if args.no_stop:
        print("Leaving PWMTEST active because --no-stop was set.")
    else:
        stop_chassis(port)


def run_stop(port: serial.Serial, args: argparse.Namespace) -> None:
    stop_chassis(port)
    if args.monitor:
        read_feedback(port, args.duration)


def run_listen(port: serial.Serial, args: argparse.Namespace) -> None:
    read_feedback(port, args.duration)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Migrated chassis USART1 bring-up tester")
    parser.add_argument("--port", required=True, help="serial port, for example COM16")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="baud rate, default 115200")
    parser.add_argument("--mode", choices=("velocity", "pwmtest", "stop", "listen"), default="velocity")
    parser.add_argument("--duration", type=float, default=1.0, help="test/listen duration in seconds")
    parser.add_argument("--monitor", action="store_true", help="print firmware feedback while running")
    parser.add_argument("--no-stop", action="store_true", help="leave the command active instead of stopping at the end")
    parser.add_argument("--dry-run", action="store_true", help="print intended action without opening serial port")

    parser.add_argument("--pattern", choices=("forward", "strafe", "rotate", "square"), default="forward")
    parser.add_argument("--vx", type=float, default=0.10, help="forward speed in m/s")
    parser.add_argument("--vy", type=float, default=0.10, help="strafe speed in m/s")
    parser.add_argument("--wz", type=float, default=0.30, help="yaw speed in rad/s")
    parser.add_argument("--set", action="append", default=[], help="runtime parameter id=value, repeatable")
    parser.add_argument("--commit", action="store_true", help="send COMMIT after --set commands")

    parser.add_argument("--motor", type=int, help="PWMTEST motor id, 0..3")
    parser.add_argument("--direction", type=int, help="PWMTEST direction: 1 or -1")
    parser.add_argument("--duty", type=float, default=1.0, help="PWMTEST duty, 0.0..1.0")
    args = parser.parse_args(argv)

    set_commands = parse_set_args(args.set)
    if args.dry_run:
        print(f"mode={args.mode}, port={args.port}, baud={args.baud}")
        print("motor map: 0=left-front, 1=left-rear, 2=right-rear, 3=right-front")
        for command in set_commands:
            print(f">> {command}")
        if args.commit:
            print(">> COMMIT")
        return 0

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0) as port:
        time.sleep(0.2)
        send_commands(port, set_commands)
        if args.commit:
            send_line(port, "COMMIT")
        try:
            if args.mode == "velocity":
                run_velocity(port, args)
            elif args.mode == "pwmtest":
                run_pwmtest(port, args)
            elif args.mode == "stop":
                run_stop(port, args)
            elif args.mode == "listen":
                run_listen(port, args)
        except KeyboardInterrupt:
            print("Interrupted, stopping chassis.")
            stop_chassis(port)
            return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
