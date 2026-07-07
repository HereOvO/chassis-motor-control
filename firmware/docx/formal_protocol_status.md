# Formal Chassis Protocol Status

This document records the current STM32 firmware protocol state after aligning with `????-?.md`.

## Active Default

The firmware currently builds and runs the formal binary protocol by default:

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 0U
```

This record applies to the current stripped motor-control firmware only.

- IMU / `MPU6050` support is not part of the active source tree.
- `PCA9685` servo support is not part of the active source tree.
- The original generic `SERVO` / `POSE` protocol is preserved only for comparison in `../LEGACY_PROTOCOL_REFERENCE.md`.

Location:

```text
Core/Inc/chassis_config.h
```

Set the macro to `1U` only when ASCII should be the boot-time default after reset. Runtime protocol mode can now be switched without rebuilding.

## Serial Link

```text
Port on test bench: COM16
Target upper-computer device: /dev/carserial
Baud: 115200
Format: 8N1, raw, no flow control
STM32 UART: USART1, PA9 TX, PA10 RX
```

## Upper Computer To STM32

### Initialization Frame

The formal protocol accepts the 9-byte initialization frame sent by `cmd_vel_sender` at startup:

```text
11 00 00 00 00 00 00 00 00
```

Firmware behavior:

```text
reset odometry
stop chassis
clear motor outputs and PID integrators
```

### Velocity Frame

The formal velocity command frame is fixed at 12 bytes:

```text
AA BB 0A 12 02 XL XH YL YH ZL ZH 00
```

Encoding:

```text
X = int16(linear.x  * 1000), little-endian
Y = int16(linear.y  * 1000), little-endian
Z = int16(angular.z * 1000), little-endian
```

Examples:

```text
forward 0.5 m/s:  AA BB 0A 12 02 F4 01 00 00 00 00 00
backward 0.5 m/s: AA BB 0A 12 02 0C FE 00 00 00 00 00
stop:             AA BB 0A 12 02 00 00 00 00 00 00 00
```

Dedicated runtime protocol-switch frame:

```text
FE EF 50 MM CC FD
MM = 00 -> ASCII
MM = 01 -> MOWEN
CC = (0x50 + MM) & 0xFF
example ASCII switch: FE EF 50 00 50 FD
example MOWEN switch: FE EF 50 01 51 FD
```

This switch frame is decoded before normal ASCII or MOWEN traffic. The `FE EF` prefix does not overlap the formal `AA BB ...` header or the `11 00 ...` init frame, and it does not overlap normal printable ASCII command traffic.

Firmware behavior after a valid velocity frame:

```text
enter velocity mode automatically
apply mecanum inverse kinematics
run per-wheel PID + feedforward
output PWM through the calibrated motor map
```

## Current Default Per-Wheel PID Profiles

The current source default PID/feedforward parameters were updated from actual bench test results on `2026-07-07`.

| Motor ID | Wheel | kp | ki | kd | kv | k_static | output_limit | integral_limit | deadband_rpm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | Left-front | `60.0` | `8.0` | `0.7` | `28.8` | `6800.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| 1 | Left-rear | `50.0` | `10.0` | `0.5` | `36.74` | `7400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| 2 | Right-rear | `110.0` | `8.0` | `0.5` | `37.0` | `6400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| 3 | Right-front | `110.0` | `8.0` | `0.5` | `42.0` | `5400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |

These are the compiled startup defaults in `Core/Src/motor_control.c`. Runtime `MSET` writes still override the active RAM profile after switching to `ASCII` mode.

## STM32 To Upper Computer

The formal feedback frame is fixed at 12 bytes:

```text
AA BB A1 A2 XL XH YL YH ZL ZH 00 CHECKSUM
```

Current firmware sets:

```text
A1 = 0x0A
A2 = 0x12
Byte10 = 0x00
```

Velocity feedback fields are encoder/odometry-derived chassis velocity:

```text
X = int16(vx * 1000), little-endian
Y = int16(vy * 1000), little-endian
Z = int16(wz * 1000), little-endian
```

Checksum:

```text
CHECKSUM = (A1 + A2 + XL + XH + YL + YH + ZL + ZH) & 0xFF
```

Checksum does not include:

```text
AA BB
Byte10
CHECKSUM itself
```

Zero-speed feedback example:

```text
AA BB 0A 12 00 00 00 00 00 00 00 1C
```

## Verified Test

Date: 2026-07-04
Hardware: STM32F407 + ST-Link + USART1 on COM16
Firmware: formal protocol build, `CHASSIS_USE_DEBUG_PROTOCOL=0U`

Test sequence:

```text
send init frame: 11 00 00 00 00 00 00 00 00
send stop frame: AA BB 0A 12 02 00 00 00 00 00 00 00
send forward frame: AA BB 0A 12 02 C8 00 00 00 00 00 00  (0.2 m/s)
```

Observed feedback:

```text
AA BB 0A 12 00 00 00 00 00 00 00 1C  checksum OK
AA BB 0A 12 C4 00 17 00 CE 01 00 C6  checksum OK
AA BB 0A 12 B2 00 17 00 46 01 00 2C  checksum OK
```

Result: formal protocol receive, automatic velocity mode entry, binary feedback framing, and checksum generation are working on COM16.

## Debug Protocol Is Still Available

The ASCII debug protocol is preserved and can be entered at runtime by the sideband switch frame above. The build macro only selects the boot-time default:

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 1U
```

Debug-only capabilities include:

```text
VEL,<vx>,<vy>,<wz>
PWMTEST,<motor_id>,<direction>,<duty>
MSET,<motor_id>,<param_id>,<value>
SET,<param_id>,<value>
```

These commands are not active in the current formal protocol firmware build.
