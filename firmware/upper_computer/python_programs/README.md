# Chassis USART1 Bring-up Tools

## Current Protocol State

Current flashed firmware and source default use the formal binary protocol:

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 0U
```

Formal protocol should be tested with `--mode mowen` or raw binary frames. ASCII commands such as `VEL`, `PWMTEST`, `MSET`, and `SET` are still preserved in firmware. `CHASSIS_USE_DEBUG_PROTOCOL` now only changes the boot-time default:

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 1U
```

Formal protocol frames:

```text
init:  11 00 00 00 00 00 00 00 00
cmd:   AA BB 0A 12 02 XL XH YL YH ZL ZH 00
fb:    AA BB 0A 12 XL XH YL YH ZL ZH 00 CHECKSUM
cksum: sum(fb[2]..fb[9]) & 0xFF
```

Dedicated runtime protocol-switch frame:

```text
switch: FE EF 50 MM CC FD
MM=00 -> ASCII
MM=01 -> MOWEN
CC=(0x50 + MM) & 0xFF
ASCII switch example: FE EF 50 00 50 FD
MOWEN switch example: FE EF 50 01 51 FD
```

COM16 formal protocol test command:

```powershell
python chassis_bringup_test.py --port COM16 --mode mowen --pattern forward --vx 0.2 --duration 2
```
This directory contains PC-side helper scripts for the migrated STM32F407 chassis firmware.
The current bring-up path uses USART1 at 115200 baud. In the present test bench the USB-UART is `COM16`.

The older generic `SERVO` / `POSE` protocol is preserved only as a comparison reference in `..\LEGACY_PROTOCOL_REFERENCE.md`. It is not part of the current USART1 chassis control path.

## Main Script

Use `chassis_bringup_test.py` for direct firmware tests:

```powershell
python chassis_bringup_test.py --port COM16 --mode stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern forward --vx 0.8 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern left --vy 0.5 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern right --vy 0.5 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode pwmtest --motor 2 --direction -1 --duty 1.0 --duration 10000 --no-stop
```

Always send stop before changing long-running motion commands:

```powershell
python chassis_bringup_test.py --port COM16 --mode stop
```


## Protocol Boot Default

Firmware selects the power-up default UART protocol in `Core/Inc/chassis_config.h`:

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 1U
```

Use `1U` when the board should boot directly into ASCII debug mode. Use `0U` when it should boot directly into the MOWEN production binary protocol described in `MOWEN??????????.md`.

If the board is already flashed, runtime switching does not require a rebuild. Send the dedicated sideband switch frame shown above.

Debug protocol input is ASCII, for example:

```text
VEL,0.800,0.000,0.000
PWMTEST,2,-1,1.000
```

MOWEN production input is a 12-byte binary frame:

```text
[0]  0xAA
[1]  0xBB
[2]  0x0A
[3]  0x12
[4]  0x02
[5]  vx low byte   int16 little-endian, m/s * 1000
[6]  vx high byte
[7]  vy low byte   int16 little-endian, m/s * 1000
[8]  vy high byte
[9]  wz low byte   int16 little-endian, rad/s * 1000
[10] wz high byte
[11] 0x00
```

MOWEN/formal production feedback is also 12 bytes:

```text
AA BB 0A 12 vx_l vx_h vy_l vy_h wz_l wz_h 00 checksum
```

The feedback checksum is `sum(frame[2]..frame[9]) & 0xFF`, matching the protocol note.

The test script can dry-run or send MOWEN frames:

```powershell
python chassis_bringup_test.py --port COM16 --mode mowen --pattern forward --vx 0.8 --dry-run
python chassis_bringup_test.py --port COM16 --mode mowen --pattern right --vy 0.5 --duration 10000 --no-stop
```
## Firmware Commands Used

The script sends ASCII commands terminated by CRLF:

```text
ASCII
ZERO
PROFILE,1
MODE,1
ENABLE,1
VEL,<vx_mps>,<vy_mps>,<wz_radps>
PWMTEST,<motor_id>,<direction>,<duty>
RAWPWM,<motor_id>,<direction>,<duty>
```

These ASCII commands become active when runtime mode is switched to `ASCII`, even if the current firmware booted in `MOWEN` mode.

Velocity convention in the current firmware:

```text
vx > 0: forward
vx < 0: backward
vy > 0: left strafe
vy < 0: right strafe
wz > 0: left turn
wz < 0: right turn
```

## Calibrated Motor Mapping

The firmware and scripts use this motor index order:

```text
0 = left-front wheel
1 = left-rear wheel
2 = right-rear wheel
3 = right-front wheel
```

Validated wheel directions with the current firmware and wiring:

```text
forward:    0/1/2/3 = forward / forward / forward / forward
backward:   0/1/2/3 = backward / backward / backward / backward
turn-left:  0/1/2/3 = backward / backward / forward / forward
turn-right: 0/1/2/3 = forward / forward / backward / backward
left:       0/1/2/3 = backward / forward / backward / forward
right:      0/1/2/3 = forward / backward / forward / backward
```

PWM and encoder wiring currently recorded in firmware:

```text
M0 left-front:  forward PE9/TIM1_CH1,  reverse PE5/TIM9_CH1,   encoder TIM2, direction_invert=1
M1 left-rear:   forward PE11/TIM1_CH2, reverse PE6/TIM9_CH2,   encoder TIM3, direction_invert=1
M2 right-rear:  forward PE13/TIM1_CH3, reverse PB14/TIM12_CH1, encoder TIM4, direction_invert=-1
M3 right-front: forward PE14/TIM1_CH4, reverse PB15/TIM12_CH2, encoder TIM5, direction_invert=-1
```

## Useful Test Commands

Dry-run without opening the serial port:

```powershell
python chassis_bringup_test.py --port COM16 --pattern left --vy 0.5 --dry-run
python chassis_bringup_test.py --port COM16 --pattern right --vy 0.5 --dry-run
```

Long-running motion tests:

```powershell
python chassis_bringup_test.py --port COM16 --mode velocity --pattern forward --vx 0.8 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern backward --vx 0.8 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern left --vy 0.5 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern right --vy 0.5 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern turn-left --wz 1.5 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode velocity --pattern turn-right --wz 1.5 --duration 10000 --no-stop
```

Raw PWM motor tests that bypass PID and encoder feedback:

```powershell
python chassis_bringup_test.py --port COM16 --mode pwmtest --motor 0 --direction 1 --duty 1.0 --duration 10000 --no-stop
python chassis_bringup_test.py --port COM16 --mode pwmtest --motor 0 --direction -1 --duty 1.0 --duration 10000 --no-stop
```

Listen to firmware feedback:

```powershell
python chassis_bringup_test.py --port COM16 --mode listen --duration 5
```

## Notes

The older `serial_server.py`, `sender_client.py`, and `receiver_client.py` files are legacy ZeroMQ tools from the original project. They are not required for the current USART1 bring-up path.

The historical `SERVO` / `POSE` protocol text is also preserved for comparison. The related reference-side scripts such as `sender_client.py`, `receiver_client.py`, and `protocol_test.py` should be treated as legacy comparison material, not as the active firmware interface.

If no PWM waveform is visible on an output pin, first check whether the command requested 100% duty. A 100% compare value can appear as a static high level instead of a square wave.


## Per-Motor PID and Feedforward Tuning

Each motor has an independent control profile in firmware. The ASCII debug protocol supports online tuning with:

```text
MSET,<motor_id>,<param_id>,<value>
```

Motor ids:

```text
0 = left-front
1 = left-rear
2 = right-rear
3 = right-front
```

Parameter ids:

```text
0 = kp
1 = ki
2 = kd
3 = kv          feedforward velocity gain
4 = k_static    static feedforward/friction offset
5 = output_limit
6 = integral_limit
7 = deadband_rpm
```

Current compiled startup defaults:

```text
motor 0 left-front:  kp=60.0   ki=8.0  kd=0.7  kv=28.8   k_static=6800.0  output_limit=MOTOR_PWM_ARR  integral_limit=300.0  deadband_rpm=1.0
motor 1 left-rear:   kp=50.0   ki=10.0 kd=0.5  kv=36.74  k_static=7400.0  output_limit=MOTOR_PWM_ARR  integral_limit=300.0  deadband_rpm=1.0
motor 2 right-rear:  kp=110.0  ki=8.0  kd=0.5  kv=37.0   k_static=6400.0  output_limit=MOTOR_PWM_ARR  integral_limit=300.0  deadband_rpm=1.0
motor 3 right-front: kp=110.0  ki=8.0  kd=0.5  kv=42.0   k_static=5400.0  output_limit=MOTOR_PWM_ARR  integral_limit=300.0  deadband_rpm=1.0
```

Examples:

```powershell
python chassis_bringup_test.py --port COM16 --mset 0:kp=60 --mset 0:ki=8 --mode listen --duration 1
python chassis_bringup_test.py --port COM16 --mset 2:kp=110 --mset 2:kv=37.0 --mode velocity --pattern forward --vx 0.5 --duration 5
```

The script converts names to `MSET` commands. For direct serial tools, send for example:

```text
MSET,2,0,110
MSET,2,3,37.0
```

Firmware feedback lines include the current values as `kp/ki/kd/kv/ks/olim/ilim/db` for each motor, so parameter writes can be confirmed from UART output.

