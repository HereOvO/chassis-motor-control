#!/usr/bin/env python3
"""USART1 protocol switch and communication smoke test.

This helper sends the dedicated sideband protocol-switch frame, ASCII debug
commands, and formal MOWEN binary frames over a serial port.

Action options are executed in the exact order they appear on the command line.
Quote ASCII commands in PowerShell so commas stay inside one argument.

Examples:

    python protocol_switch_test.py --port COM16 --switch ascii --ascii "ZERO"

    python protocol_switch_test.py --port COM16 --switch ascii --ascii "MODE,1" \
        --ascii "VEL,0.200,0.000,0.000"

    python protocol_switch_test.py --port COM16 --switch ascii --ascii "ZERO" \
        --switch mowen --mowen-init --mowen-vel 0.200 0.000 0.000
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc


DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 0.10
DEFAULT_WRITE_TIMEOUT = 1.0
DEFAULT_OPEN_DELAY = 0.20
DEFAULT_POST_TX_DELAY = 0.05
DEFAULT_RX_WINDOW = 0.30

SWITCH_MODE_BYTES = {
    "ascii": 0x00,
    "mowen": 0x01,
}


@dataclass(frozen=True)
class Action:
    kind: str
    value: Any


def build_parser() -> argparse.ArgumentParser:
    epilog = """Action syntax, executed in order:
  --switch ascii|mowen
  --ascii "ZERO"
  --ascii "MODE,1"
  --ascii "VEL,0.200,0.000,0.000"
  --ascii "MSET,2,0,110"
  --mowen-init
  --mowen-vel VX VY WZ
  --listen SECONDS
  --sleep SECONDS

Example full sequence:
  python protocol_switch_test.py --port COM16 --switch ascii --ascii "ZERO" --switch mowen --mowen-vel 0.200 0.000 0.000
"""
    parser = argparse.ArgumentParser(
        description="Serial tester for runtime protocol switching and basic UART communication",
        epilog=epilog,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", help="serial port, for example COM16")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="baud rate, default 115200")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="serial read timeout in seconds")
    parser.add_argument(
        "--write-timeout",
        type=float,
        default=DEFAULT_WRITE_TIMEOUT,
        help="serial write timeout in seconds",
    )
    parser.add_argument(
        "--open-delay",
        type=float,
        default=DEFAULT_OPEN_DELAY,
        help="delay after opening the port before the first action",
    )
    parser.add_argument(
        "--post-tx-delay",
        type=float,
        default=DEFAULT_POST_TX_DELAY,
        help="delay after each transmit before reading feedback",
    )
    parser.add_argument(
        "--rx-window",
        type=float,
        default=DEFAULT_RX_WINDOW,
        help="default read window after each transmit in seconds, use 0 to skip",
    )
    parser.add_argument("--dry-run", action="store_true", help="print actions without opening the serial port")
    parser.add_argument("--list-ports", action="store_true", help="list available serial ports and exit")
    return parser


def parse_actions(tokens: list[str]) -> list[Action]:
    actions: list[Action] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if token == "--switch":
            mode = require_token(tokens, index + 1, "--switch requires ascii or mowen")
            if mode not in SWITCH_MODE_BYTES:
                raise ValueError(f"unsupported switch mode {mode!r}, expected ascii or mowen")
            actions.append(Action("switch", mode))
            index += 2
            continue
        if token == "--ascii":
            command = require_token(tokens, index + 1, "--ascii requires a command string")
            actions.append(Action("ascii", command))
            index += 2
            continue
        if token == "--mowen-init":
            actions.append(Action("mowen_init", None))
            index += 1
            continue
        if token == "--mowen-vel":
            vx_text = require_token(tokens, index + 1, "--mowen-vel requires VX VY WZ")
            vy_text = require_token(tokens, index + 2, "--mowen-vel requires VX VY WZ")
            wz_text = require_token(tokens, index + 3, "--mowen-vel requires VX VY WZ")
            try:
                velocity = (float(vx_text), float(vy_text), float(wz_text))
            except ValueError as exc:
                raise ValueError("--mowen-vel expects numeric VX VY WZ") from exc
            actions.append(Action("mowen_vel", velocity))
            index += 4
            continue
        if token == "--listen":
            seconds_text = require_token(tokens, index + 1, "--listen requires seconds")
            try:
                seconds = float(seconds_text)
            except ValueError as exc:
                raise ValueError("--listen expects a numeric duration in seconds") from exc
            actions.append(Action("listen", seconds))
            index += 2
            continue
        if token == "--sleep":
            seconds_text = require_token(tokens, index + 1, "--sleep requires seconds")
            try:
                seconds = float(seconds_text)
            except ValueError as exc:
                raise ValueError("--sleep expects a numeric duration in seconds") from exc
            actions.append(Action("sleep", seconds))
            index += 2
            continue
        raise ValueError(f"unknown action token {token!r}")
    return actions


def require_token(tokens: list[str], index: int, error_message: str) -> str:
    if index >= len(tokens):
        raise ValueError(error_message)
    return tokens[index]


def clamp_i16_scaled(value: float) -> int:
    scaled = int(round(value * 1000.0))
    return max(-32768, min(32767, scaled))


def build_switch_frame(mode: str) -> bytes:
    mode_byte = SWITCH_MODE_BYTES[mode]
    checksum = (0x50 + mode_byte) & 0xFF
    return bytes((0xFE, 0xEF, 0x50, mode_byte, checksum, 0xFD))


def build_mowen_init_frame() -> bytes:
    return bytes((0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))


def build_mowen_velocity_frame(vx: float, vy: float, wz: float) -> bytes:
    return b"\xAA\xBB\x0A\x12\x02" + struct.pack(
        "<hhhB",
        clamp_i16_scaled(vx),
        clamp_i16_scaled(vy),
        clamp_i16_scaled(wz),
        0x00,
    )


def build_ascii_payload(command: str) -> bytes:
    stripped = command.strip()
    if not stripped:
        raise ValueError("ASCII command must not be empty")
    try:
        return (stripped + "\r\n").encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError(f"ASCII command contains non-ASCII characters: {command!r}") from exc


def format_hex(data: bytes) -> str:
    return data.hex(" ").upper()


def format_ascii_preview(data: bytes) -> str:
    preview: list[str] = []
    for byte in data:
        if byte == 0x0D:
            preview.append("\\r")
        elif byte == 0x0A:
            preview.append("\\n")
        elif 32 <= byte <= 126:
            preview.append(chr(byte))
        else:
            preview.append(f"\\x{byte:02X}")
    return "".join(preview)


def print_rx(data: bytes) -> None:
    print(f"<< HEX   {format_hex(data)}")
    print(f"<< ASCII {format_ascii_preview(data)}")


def read_for(port: serial.Serial, seconds: float) -> bool:
    deadline = time.monotonic() + max(0.0, seconds)
    saw_data = False
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        chunk = port.read(waiting or 1)
        if not chunk:
            continue
        saw_data = True
        print_rx(chunk)
    while port.in_waiting:
        saw_data = True
        print_rx(port.read(port.in_waiting))
    return saw_data


def transmit(port: serial.Serial, payload: bytes, label: str, args: argparse.Namespace) -> None:
    print(f">> {label}")
    print(f">> HEX   {format_hex(payload)}")
    if label.startswith("ASCII "):
        print(f">> ASCII {label[6:]}")
    port.write(payload)
    port.flush()
    if args.post_tx_delay > 0.0:
        time.sleep(args.post_tx_delay)
    if args.rx_window > 0.0:
        if not read_for(port, args.rx_window):
            print(f"<< (no data within {args.rx_window:.3f}s)")


def run_action(port: serial.Serial, action: Action, args: argparse.Namespace) -> None:
    if action.kind == "switch":
        mode = str(action.value)
        transmit(port, build_switch_frame(mode), f"SWITCH {mode.upper()}", args)
        return
    if action.kind == "ascii":
        command = str(action.value).strip()
        transmit(port, build_ascii_payload(command), f"ASCII {command}", args)
        return
    if action.kind == "mowen_init":
        transmit(port, build_mowen_init_frame(), "MOWEN INIT", args)
        return
    if action.kind == "mowen_vel":
        vx, vy, wz = action.value
        payload = build_mowen_velocity_frame(vx, vy, wz)
        transmit(port, payload, f"MOWEN VEL vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}", args)
        return
    if action.kind == "listen":
        seconds = float(action.value)
        print(f">> LISTEN {seconds:.3f}s")
        if not read_for(port, seconds):
            print(f"<< (no data within {seconds:.3f}s)")
        return
    if action.kind == "sleep":
        seconds = float(action.value)
        print(f">> SLEEP {seconds:.3f}s")
        time.sleep(max(0.0, seconds))
        return
    raise ValueError(f"unsupported action kind {action.kind!r}")


def dry_run(actions: list[Action], args: argparse.Namespace) -> int:
    print(f"port={args.port or '<not opened>'} baud={args.baud}")
    print("dry-run only; no serial port will be opened.")
    for action in actions:
        if action.kind == "switch":
            mode = str(action.value)
            print(f">> SWITCH {mode.upper()}")
            print(f">> HEX   {format_hex(build_switch_frame(mode))}")
        elif action.kind == "ascii":
            command = str(action.value).strip()
            payload = build_ascii_payload(command)
            print(f">> ASCII {command}")
            print(f">> HEX   {format_hex(payload)}")
        elif action.kind == "mowen_init":
            print(">> MOWEN INIT")
            print(f">> HEX   {format_hex(build_mowen_init_frame())}")
        elif action.kind == "mowen_vel":
            vx, vy, wz = action.value
            print(f">> MOWEN VEL vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")
            print(f">> HEX   {format_hex(build_mowen_velocity_frame(vx, vy, wz))}")
        elif action.kind == "listen":
            print(f">> LISTEN {float(action.value):.3f}s")
        elif action.kind == "sleep":
            print(f">> SLEEP {float(action.value):.3f}s")
    return 0


def print_ports() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 0
    for item in ports:
        description = item.description or "unknown"
        hwid = item.hwid or "n/a"
        print(f"{item.device}: {description} [{hwid}]")
    return 0


def main(argv: list[str]) -> int:
    parser = build_parser()
    args, action_tokens = parser.parse_known_args(argv)

    if args.list_ports:
        return print_ports()

    try:
        actions = parse_actions(action_tokens)
    except ValueError as exc:
        parser.print_usage(sys.stderr)
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if not actions:
        parser.print_usage(sys.stderr)
        print("error: no actions specified", file=sys.stderr)
        return 2

    if args.dry_run:
        return dry_run(actions, args)

    if not args.port:
        parser.print_usage(sys.stderr)
        print("error: --port is required unless --dry-run or --list-ports is used", file=sys.stderr)
        return 2

    with serial.Serial(
        args.port,
        args.baud,
        timeout=args.timeout,
        write_timeout=args.write_timeout,
    ) as port:
        time.sleep(max(0.0, args.open_delay))
        port.reset_input_buffer()
        port.reset_output_buffer()
        for action in actions:
            run_action(port, action, args)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
