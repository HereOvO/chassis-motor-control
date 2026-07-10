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

Validated wheel directions with current firmware and wiring:

    forward:   0/1/2/3 = forward/forward/forward/forward
    backward:  0/1/2/3 = backward/backward/backward/backward
    turn-left: 0/1/2/3 = backward/backward/forward/forward
    turn-right:0/1/2/3 = forward/forward/backward/backward
    left:      0/1/2/3 = backward/forward/backward/forward
    right:     0/1/2/3 = forward/backward/forward/backward
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc


DEFAULT_BAUD = 115200
COMMAND_REFRESH_PERIOD_S = 0.05
MOTOR_LABELS = {
    0: "left-front",
    1: "left-rear",
    2: "right-rear",
    3: "right-front",
}

MOTOR_PARAM_IDS = {
    "kp": 0,
    "ki": 1,
    "kd": 2,
    "kv": 3,
    "ks": 4,
    "k_static": 4,
    "output_limit": 5,
    "olim": 5,
    "integral_limit": 6,
    "ilim": 6,
    "deadband": 7,
    "deadband_rpm": 7,
    "db": 7,
}
EXPECTED_DIRECTIONS = {
    "forward": "0/1/2/3 = forward/forward/forward/forward",
    "backward": "0/1/2/3 = backward/backward/backward/backward",
    "turn-left": "0/1/2/3 = backward/backward/forward/forward",
    "turn-right": "0/1/2/3 = forward/forward/backward/backward",
    "left": "0/1/2/3 = backward/forward/backward/forward",
    "right": "0/1/2/3 = forward/backward/forward/backward",
}


@dataclass(frozen=True)
class VelocityStep:
    vx: float
    vy: float
    wz: float
    duration_s: float


def send_line(port: serial.Serial, line: str, delay_s: float = 0.05, announce: bool = True) -> None:
    command = line.strip()
    if announce:
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



def parse_mset_args(values: list[str]) -> list[str]:
    commands: list[str] = []
    for item in values:
        if "=" not in item or ":" not in item:
            raise ValueError(f"invalid --mset item {item!r}, expected motor:param=value")
        lhs, value = item.split("=", 1)
        motor_text, param_text = lhs.split(":", 1)
        motor_id = int(motor_text, 10)
        key = param_text.strip().lower()
        if motor_id < 0 or motor_id > 3:
            raise ValueError("--mset motor id must be 0..3")
        if key not in MOTOR_PARAM_IDS:
            raise ValueError(f"unknown motor param {param_text!r}")
        commands.append(f"MSET,{motor_id},{MOTOR_PARAM_IDS[key]},{float(value):.6g}")
    return commands

def build_steps(args: argparse.Namespace) -> list[VelocityStep]:
    if args.pattern == "forward":
        return [VelocityStep(abs(args.vx), 0.0, 0.0, args.duration)]
    if args.pattern == "backward":
        return [VelocityStep(-abs(args.vx), 0.0, 0.0, args.duration)]
    if args.pattern == "left":
        return [VelocityStep(0.0, abs(args.vy), 0.0, args.duration)]
    if args.pattern == "right":
        return [VelocityStep(0.0, -abs(args.vy), 0.0, args.duration)]
    if args.pattern == "strafe":
        return [VelocityStep(0.0, args.vy, 0.0, args.duration)]
    if args.pattern == "turn-left":
        return [VelocityStep(0.0, 0.0, abs(args.wz), args.duration)]
    if args.pattern == "turn-right":
        return [VelocityStep(0.0, 0.0, -abs(args.wz), args.duration)]
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



def clamp_i16_scaled(value: float) -> int:
    scaled = int(round(value * 1000.0))
    return max(-32768, min(32767, scaled))


def build_mowen_frame(vx: float, vy: float, wz: float) -> bytes:
    return b"\xAA\xBB\x0A\x12\x02" + struct.pack("<hhhB", clamp_i16_scaled(vx), clamp_i16_scaled(vy), clamp_i16_scaled(wz), 0)


def send_mowen_frame(port: serial.Serial, step: VelocityStep, delay_s: float = 0.05, announce: bool = True) -> None:
    frame = build_mowen_frame(step.vx, step.vy, step.wz)
    if announce:
        print(f">> MOWEN {frame.hex(' ').upper()}  VEL,{step.vx:.3f},{step.vy:.3f},{step.wz:.3f}")
    port.write(frame)
    port.flush()
    if delay_s > 0.0:
        time.sleep(delay_s)


def run_periodic_payload(
    port: serial.Serial,
    payload: bytes,
    label: str,
    duration_s: float,
    monitor: bool,
) -> None:
    print(label)
    deadline = time.monotonic() + max(0.0, duration_s)
    while True:
        port.write(payload)
        port.flush()
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        wait_s = min(COMMAND_REFRESH_PERIOD_S, remaining)
        if monitor:
            read_feedback(port, wait_s)
        else:
            time.sleep(wait_s)

def run_velocity(port: serial.Serial, args: argparse.Namespace) -> None:
    send_commands(port, ("ASCII", "ZERO", "PROFILE,1", "MODE,1", "ENABLE,1"))
    for step in build_steps(args):
        command = f"VEL,{step.vx:.3f},{step.vy:.3f},{step.wz:.3f}"
        run_periodic_payload(
            port,
            (command + "\r\n").encode("ascii"),
            f">> {command} (refresh every {COMMAND_REFRESH_PERIOD_S * 1000:.0f} ms)",
            step.duration_s,
            args.monitor,
        )
    if args.no_stop:
        print("No explicit stop sent; the firmware watchdog will stop motion after command refresh ends.")
    else:
        stop_chassis(port)



def run_mowen(port: serial.Serial, args: argparse.Namespace) -> None:
    for step in build_steps(args):
        frame = build_mowen_frame(step.vx, step.vy, step.wz)
        run_periodic_payload(
            port,
            frame,
            f">> MOWEN {frame.hex(' ').upper()}  VEL,{step.vx:.3f},{step.vy:.3f},{step.wz:.3f} "
            f"(refresh every {COMMAND_REFRESH_PERIOD_S * 1000:.0f} ms)",
            step.duration_s,
            args.monitor,
        )
    if args.no_stop:
        print("No explicit stop sent; the firmware watchdog will stop motion after command refresh ends.")
    else:
        send_mowen_frame(port, VelocityStep(0.0, 0.0, 0.0, 0.0))

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
    command = f"PWMTEST,{args.motor},{args.direction},{args.duty:.3f}"
    run_periodic_payload(
        port,
        (command + "\r\n").encode("ascii"),
        f">> {command} (refresh every {COMMAND_REFRESH_PERIOD_S * 1000:.0f} ms)",
        args.duration,
        args.monitor,
    )
    if args.no_stop:
        print("No explicit stop sent; the firmware watchdog will stop PWM after command refresh ends.")
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
    parser.add_argument("--mode", choices=("velocity", "mowen", "pwmtest", "stop", "listen"), default="velocity")
    parser.add_argument("--duration", type=float, default=1.0, help="test/listen duration in seconds")
    parser.add_argument("--monitor", action="store_true", help="print firmware feedback while running")
    parser.add_argument("--no-stop", action="store_true", help="skip the explicit stop frame; the 200 ms firmware watchdog still stops motion")
    parser.add_argument("--dry-run", action="store_true", help="print intended action without opening serial port")

    parser.add_argument("--pattern", choices=("forward", "backward", "left", "right", "strafe", "turn-left", "turn-right", "rotate", "square"), default="forward")
    parser.add_argument("--vx", type=float, default=0.10, help="forward speed in m/s")
    parser.add_argument("--vy", type=float, default=0.10, help="strafe speed in m/s")
    parser.add_argument("--wz", type=float, default=0.30, help="yaw speed in rad/s")
    parser.add_argument("--set", action="append", default=[], help="runtime chassis parameter id=value, repeatable")
    parser.add_argument("--mset", action="append", default=[], help="motor control parameter motor:param=value, e.g. 2:kp=220")
    parser.add_argument("--commit", action="store_true", help="send COMMIT after --set commands")

    parser.add_argument("--motor", type=int, help="PWMTEST motor id, 0..3")
    parser.add_argument("--direction", type=int, help="PWMTEST direction: 1 or -1")
    parser.add_argument("--duty", type=float, default=1.0, help="PWMTEST duty, 0.0..1.0")
    args = parser.parse_args(argv)

    set_commands = parse_set_args(args.set) + parse_mset_args(args.mset)
    if args.dry_run:
        print(f"mode={args.mode}, port={args.port}, baud={args.baud}")
        print("motor map: 0=left-front, 1=left-rear, 2=right-rear, 3=right-front")
        if args.mode in ("velocity", "mowen"):
            for step in build_steps(args):
                if args.mode == "mowen":
                    frame_hex = build_mowen_frame(step.vx, step.vy, step.wz).hex(" ").upper()
                    print(f">> MOWEN {frame_hex}  VEL,{step.vx:.3f},{step.vy:.3f},{step.wz:.3f}")
                else:
                    print(f">> VEL,{step.vx:.3f},{step.vy:.3f},{step.wz:.3f}")
            if args.pattern in EXPECTED_DIRECTIONS:
                print(f"expected: {EXPECTED_DIRECTIONS[args.pattern]}")
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
            elif args.mode == "mowen":
                run_mowen(port, args)
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





