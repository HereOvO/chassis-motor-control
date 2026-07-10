# Wheeled Leg Robot Motor Migration Project

This document describes the current motor-drive migration firmware under
`wheeled-leg-robot/firmware`. It reflects the implemented project, not the old
sample firmware.

## Build Environment

- IDE: Keil MDK-ARM uVision 5.
- Compiler: ARMCC 5.05 update 1, installed at `D:\Keil_v5\ARM\ARMCC\Bin`.
- Project: `MDK-ARM/wheeled_leg_robot_f407.uvprojx`.
- Target: `wheeled_leg_robot_f407`.
- Verified command:

```powershell
D:\Keil_v5\UV4\uVision.com -r MDK-ARM\wheeled_leg_robot_f407.uvprojx -t wheeled_leg_robot_f407 -o MDK-ARM\keil_rebuild.log
```

Expected result:

```text
0 Error(s), 0 Warning(s)
```

## Scope Of Current Firmware

The active firmware tree is now restricted to chassis motor control.

- IMU / `MPU6050` support is removed from the active source tree.
- `PCA9685` servo support is removed from the active source tree.
- `I2C1` / `I2C2` generated source files are removed from the active source tree.
- `UART5` servo / pose implementation is not part of the active build.

The original generic `SERVO` / `POSE` protocol is preserved only as a comparison reference in `LEGACY_PROTOCOL_REFERENCE.md`. It is not the current firmware interface.

## Runtime Chain

The active chain is:

```text
USART1 RX interrupt
-> chassis_protocol_poll()
-> chassis_mode_apply_cmd()
-> chassis_kinematics_inverse()
-> MotorControlCore_SetTargets()
-> MotorControlCore_Compute()
-> MotorDriver_SetOutput()
-> encoder feedback
-> chassis_odometry_update()
```

The odometry module is implemented but not transmitted to the upper computer yet.
Use a debugger/watch window to inspect `g_odometry` or call
`chassis_odometry_get()` from future reporting code.

## Hardware Roles

- `USART1`: command/debug port, PA9 TX and PA10 RX.
- `TIM14`: HAL timebase.
- `TIM2/TIM3/TIM4/TIM5`: wheel encoders.
- `TIM1/TIM9/TIM12`: dual-PWM motor outputs.
- `UART5`: not part of the active Keil build.

Motor index mapping:

| Motor ID | Physical wheel |
| --- | --- |
| 0 | Left-front wheel |
| 1 | Left-rear wheel |
| 2 | Right-rear wheel |
| 3 | Right-front wheel |

All kinematics, odometry, feedback and `PWMTEST` commands use this order.

## FreeRTOS Task Model

Only one active chassis task is used for the motor chain:

- `ChassisControlTask`: 1 ms loop.
- Protocol polling: every loop.
- Encoder sampling: every `MOTOR_SPEED_SAMPLE_TIME_MS`, currently 20 ms / 50 Hz.
- Odometry update: immediately after encoder sampling, also 50 Hz.
- PID compute and motor output: every `MOTOR_PID_CONTROL_PERIOD_MS`, currently 10 ms / 100 Hz.

## Current Default Per-Motor PID Profiles

The firmware source default PID/feedforward parameters were updated from actual bench test results on `2026-07-07`.

| Motor ID | Wheel | kp | ki | kd | kv | k_static | output_limit | integral_limit | deadband_rpm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | Left-front | `60.0` | `8.0` | `0.7` | `28.8` | `6800.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| 1 | Left-rear | `50.0` | `10.0` | `0.5` | `36.74` | `7400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| 2 | Right-rear | `110.0` | `8.0` | `0.5` | `37.0` | `6400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| 3 | Right-front | `110.0` | `8.0` | `0.5` | `42.0` | `5400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |

These defaults are compiled into `Core/Src/motor_control.c`. Runtime ASCII tuning with `MSET,<motor_id>,<param_id>,<value>` is still available when the protocol is switched to `ASCII`.

## Command Protocol

Formal binary protocol is the current default boot mode (`CHASSIS_USE_DEBUG_PROTOCOL=0U`). The preserved ASCII debug protocol remains available at runtime; `CHASSIS_USE_DEBUG_PROTOCOL=1U` only changes the power-up default mode. ASCII commands are comma-separated and end with CR/LF or LF.

```text
ENABLE,1
ENABLE,0
MODE,0
MODE,1
PROFILE,1
VEL,0.20,0.00,0.00
CMD_VEL,0.20,0.00,0.00
SET,0,0.085
COMMIT
RESTORE
ZERO
ASCII
MOWEN
PWMTEST,0,1,0.30
RAWPWM,0,-1,0.30
```

Dedicated sideband runtime protocol-switch frame:

```text
FE EF 50 MM CC FD
MM = 00 -> ASCII
MM = 01 -> MOWEN
CC = (0x50 + MM) & 0xFF
example ASCII switch: FE EF 50 00 50 FD
example MOWEN switch: FE EF 50 01 51 FD
```

The switch detector starts only at an idle protocol boundary. Once a MOWEN frame has started, every payload byte is preserved as frame data, including `0xFE` in a negative signed velocity value. This prevents negative values such as `D4 FE` from being mistaken for the start of a switch frame.

Modes:

- `0`: stop.
- `1`: velocity.
- `2`: tune.
- `3`: fault.

Profiles:

- `0`: differential compatibility profile.
- `1`: mecanum main profile and default profile.


## Current Formal Binary Protocol

The current default source and flashed firmware boot into the formal binary protocol documented in the repository root new communication protocol document:

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 0U
```

Input frames on USART1:

```text
init: 11 00 00 00 00 00 00 00 00
cmd:  AA BB 0A 12 02 XL XH YL YH ZL ZH 00
```

Velocity encoding:

```text
X = int16(vx * 1000), little-endian
Y = int16(vy * 1000), little-endian
Z = int16(wz * 1000), little-endian
```

The initialization frame resets odometry and stops the chassis. A valid velocity frame automatically enters velocity mode and runs mecanum inverse kinematics.

The command frame remains exactly 12 bytes; byte 11 is the reserved trailing `0x00` and must be transmitted. Velocity and raw-PWM motion commands refresh a `200 ms` firmware watchdog. Send active motion commands every `20-100 ms`. If command refresh stops, the firmware enters stop mode, clears all wheel targets, resets the control outputs, and drives all PWM compare registers to zero.

Feedback frame:

```text
AA BB 0A 12 XL XH YL YH ZL ZH 00 CHECKSUM
```

Checksum:

```text
CHECKSUM = (0x0A + 0x12 + XL + XH + YL + YH + ZL + ZH) & 0xFF
```

The formal protocol was flashed and verified on COM16 after the update. Zero-speed feedback was observed as:

```text
AA BB 0A 12 00 00 00 00 00 00 00 1C
```

See `docx/formal_protocol_status.md` for the detailed protocol record and test notes.
## Runtime Tunable Parameters

`SET,<param_id>,<value>` updates the active profile in RAM. `COMMIT` commits to
the RAM committed copy only; it does not write flash.

| ID | Parameter | Unit / Values |
| --- | --- | --- |
| 0 | wheel radius | m |
| 1 | track width | m |
| 2 | wheel base | m |
| 3 | reduction ratio | ratio |
| 4 | encoder PPR | pulses/rev before quadrature |
| 5 | motor 0 direction sign | `-1` or `1` |
| 6 | motor 1 direction sign | `-1` or `1` |
| 7 | motor 2 direction sign | `-1` or `1` |
| 8 | motor 3 direction sign | `-1` or `1` |
| 9 | max linear speed | m/s |
| 10 | max angular speed | rad/s |
| 11 | max linear accel | m/s^2 |
| 12 | max angular accel | rad/s^2 |
| 13 | PWM limit | timer counts |

Per-motor PID and feedforward parameters are exposed in the preserved ASCII debug protocol with `MSET,<motor_id>,<param_id>,<value>`. When the firmware boots in formal mode, use the dedicated sideband switch frame first if runtime ASCII tuning is required.

## Raw PWM Test Command

`PWMTEST,<motor_id>,<direction>,<duty>` and `RAWPWM,<motor_id>,<direction>,<duty>`
enter a raw PWM debug mode. In this mode the firmware bypasses inverse
kinematics, PID and encoder feedback. It directly drives one motor channel until
`ZERO`, `ENABLE,0`, `MODE,0`, or a velocity command exits raw mode.

Arguments:

- `motor_id`: `0..3`, using `0=left-front`, `1=left-rear`, `2=right-rear`, `3=right-front`.
- `direction`: `1` for forward side, `-1` for reverse side, `0` for stop.
- `duty`: normalized duty `0.0..1.0`.

Examples:

```text
PWMTEST,0,1,0.30
PWMTEST,0,-1,0.30
PWMTEST,1,1,0.30
PWMTEST,1,-1,0.30
PWMTEST,2,1,0.30
PWMTEST,2,-1,0.30
PWMTEST,3,1,0.30
PWMTEST,3,-1,0.30
ZERO
```

Use this command to verify PWM pins before closed-loop testing.

## USART1 Feedback

Feedback format now follows the active runtime protocol mode. Feedback is generated
in the FreeRTOS task context, not in the UART interrupt.

When runtime mode is `ASCII`, the firmware sends ASCII feedback every 100 ms.

Two lines are emitted per report cycle:

```text
STAT,mode=1,profile=1,valid=1,x=0.0000,y=0.0000,yaw=0.0000,vx=0.0000,vy=0.0000,wz=0.0000,upd=10
MOTOR,t=0.0/0.0/0.0/0.0,rpm=0.0/0.0/0.0/0.0,delta=0/0/0/0,dir=0/0/0/0,duty=0.000/0.000/0.000/0.000,ccr=0:0/0:0/0:0/0:0
```

Fields:

- `mode/profile`: active chassis state.
- `valid`: odometry validity flag.
- `x/y/yaw`: integrated odometry.
- `vx/vy/wz`: latest encoder-derived body velocity.
- `t`: target wheel RPM after inverse kinematics.
- `rpm`: measured wheel RPM.
- `delta`: latest encoder count delta per wheel.
- `dir`: last motor driver output direction.
- `duty`: last normalized motor driver output.
- `ccr`: last forward and reverse compare values per motor.

Use the `MOTOR` line to locate no-PWM faults. If `t` is non-zero but `duty` and
`ccr` remain zero, the command/control path is not producing output. If `ccr` is
non-zero but the pin has no waveform, inspect timer PWM initialization and pin
mapping.

When runtime mode is `MOWEN`, the firmware sends the 12-byte binary feedback frame
documented in the formal protocol section above.

## Odometry

The new odometry module lives in:

- `Core/Inc/chassis_odometry.h`
- `Core/Src/chassis_odometry.c`

Inputs:

- Active `chassis_profile_t` from `runtime_tune_get_active_profile()`.
- Four `Motor_Status_t` snapshots from encoder sampling.
- Fixed update period `MOTOR_SPEED_SAMPLE_PERIOD_S`.

Outputs in `chassis_odometry_t`:

- `x_m`, `y_m`, `yaw_rad`: integrated map-frame pose.
- `vx_mps`, `vy_mps`, `wz_radps`: body-frame velocity from the latest encoder delta.
- `wheel_distance_m[4]`: accumulated wheel travel after direction sign correction.
- `wheel_total_count[4]`: raw accumulated encoder counts from motor feedback.
- `profile_id`, `drive_mode`, `update_count`, `valid`.

Reset behavior:

- Firmware init resets odometry.
- `ZERO` resets odometry.
- `RESTORE` resets odometry.
- Profile switching resets odometry to avoid mixing geometry models.

## Current Limitations

- Odometry is not sent to the upper computer yet.
- Mecanum odometry assumes wheel order consistent with the inverse kinematics in
  `chassis_kinematics.c`.
- Encoder direction signs must be verified on the real chassis before trusting
  pose direction.
- `max_linear_accel` and `max_angular_accel` are stored as profile parameters,
  but acceleration limiting is not implemented yet.
- PID parameters exist inside `motor_control.c`, but are not online tunable yet.

## Suggested Bring-Up Order

1. Flash the generated `wheeled_leg_robot_f407.hex`.
2. Connect the upper computer to `USART1` at 115200 8N1.
3. Send `ZERO`, then `PROFILE,1`, then `ENABLE,1`.
4. Test low-speed forward motion with `VEL,0.10,0.00,0.00`.
5. Stop with `VEL,0.00,0.00,0.00`, then `ENABLE,0`.
6. Check encoder deltas and `chassis_odometry_get()` in debugger.
7. Adjust direction signs with `SET,5..8,-1/1` if wheel directions are inverted.


