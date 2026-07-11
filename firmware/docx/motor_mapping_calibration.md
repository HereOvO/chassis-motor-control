# Motor Mapping And Calibration Record

Date: 2026-07-04

Project path:

```text
D:\projects_D\code\stm32_code_MX\wheeled-leg-robot-master\wheeled-leg-robot-master\wheeled-leg-robot\firmware
```

## Hardware Parameters

The current firmware defaults are based on `firmware/param/param.md` and the live calibration results.

| Item | Value |
| --- | --- |
| Wheel radius | `0.0350 m` / `35.0 mm` |
| Track width | `0.250 m` / `25 cm` |
| Wheel base | `0.220 m` / `22 cm` |
| Reduction ratio | `30.0` |
| Encoder PPR | `500` |
| Encoder quadrature multiplier | `4` |
| Output pulses per wheel revolution | `500 * 4 * 30 = 60000` |
| Max wheel speed target | `500 rpm` |
| PWM ARR / limit | `13999` |

## Motor Index Mapping

The project uses this motor order everywhere: kinematics, odometry, UART feedback, and `PWMTEST`.

| Motor ID | Physical wheel | Side |
| --- | --- | --- |
| `0` | Left-front wheel | Left |
| `1` | Left-rear wheel | Left |
| `2` | Right-rear wheel | Right |
| `3` | Right-front wheel | Right |

## PWM And Encoder Mapping

Current firmware source: `Core/Src/motor_driver.c`.

| Motor ID | Forward PWM | Reverse PWM | Encoder timer | `direction_invert` |
| --- | --- | --- | --- | --- |
| `0` | `PE9 / TIM1_CH1` | `PE5 / TIM9_CH1` | `TIM2` | `1` |
| `1` | `PE11 / TIM1_CH2` | `PE6 / TIM9_CH2` | `TIM3` | `1` |
| `2` | `PE13 / TIM1_CH3` | `PB14 / TIM12_CH1` | `TIM4` | `-1` |
| `3` | `PE14 / TIM1_CH4` | `PB15 / TIM12_CH2` | `TIM5` | `-1` |

`direction_invert = -1` means the motor driver's logical forward/reverse command is swapped before writing PWM. The encoder feedback sign is also corrected with this same flag in `Core/Src/motor_feedback.c`, so closed-loop feedback follows the logical motor direction.

## Current Tested Default PID Profiles

The source default PID/feedforward parameters were updated from actual bench tuning results on `2026-07-07`.

| Motor ID | Wheel | kp | ki | kd | kv | k_static | output_limit | integral_limit | deadband_rpm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0` | Left-front | `60.0` | `8.0` | `0.7` | `28.8` | `6800.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| `1` | Left-rear | `50.0` | `10.0` | `0.5` | `36.74` | `7400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| `2` | Right-rear | `110.0` | `8.0` | `0.5` | `37.0` | `6400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |
| `3` | Right-front | `110.0` | `8.0` | `0.5` | `42.0` | `5400.0` | `MOTOR_PWM_ARR` | `300.0` | `1.0` |

These values are the power-up defaults compiled into `Core/Src/motor_control.c`. Online tuning through `MSET` still works on the active RAM profile in `ASCII` mode.

## Raw PWM Test Results

Test command format:

```text
PWMTEST,<motor_id>,<direction>,<duty>
```

Where `direction=1` is firmware logical forward side, `direction=-1` is firmware logical reverse side, and `duty=1.0` is 100% normalized duty.

Observed physical directions before final closed-loop validation:

| Command | Observed result |
| --- | --- |
| `PWMTEST,0,1,1.0` | Motor 0 forward |
| `PWMTEST,0,-1,1.0` | Motor 0 reverse |
| `PWMTEST,1,1,1.0` | Motor 1 forward |
| `PWMTEST,1,-1,1.0` | Motor 1 reverse |
| `PWMTEST,2,1,1.0` | Motor 2 reverse |
| `PWMTEST,2,-1,1.0` | Motor 2 forward |
| `PWMTEST,3,1,1.0` | Motor 3 reverse |
| `PWMTEST,3,-1,1.0` | Motor 3 forward |

Final firmware compensates motor 2 and motor 3 with `direction_invert = -1`.

## Chassis Motion Validation

Validated after applying motor 2 and motor 3 direction inversion, wheel order correction, and encoder feedback sign correction.

Wheel order in the observation list is `0, 1, 2, 3` = left-front, left-rear, right-rear, right-front.

| Command | Expected physical wheel direction | Observed result | Status |
| --- | --- | --- | --- |
| `VEL,0.800,0.000,0.000` | all forward | forward, forward, forward, forward | Pass |
| `VEL,-0.800,0.000,0.000` | all reverse | reverse, reverse, reverse, reverse | Pass |
| `VEL,0.000,0.000,1.500` | left side reverse, right side forward | reverse, reverse, forward, forward | Pass |
| `VEL,0.000,0.000,-1.500` | left side forward, right side reverse | forward, forward, reverse, reverse | Pass |

## Kinematics Order

The current mecanum inverse kinematics output order is:

```text
wheel[0] = left-front
wheel[1] = left-rear
wheel[2] = right-rear
wheel[3] = right-front
```

For differential compatibility mode:

```text
left side  = motor 0 + motor 1
right side = motor 2 + motor 3
```

## Odometry Order

The current odometry integration uses the same wheel order:

```text
0 = left-front
1 = left-rear
2 = right-rear
3 = right-front
```

The encoder delta sign is corrected by `direction_invert` before speed, total count, and odometry are updated.

## Useful Test Commands

Stop chassis:

```powershell
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode stop
```

Listen to feedback:

```powershell
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode listen --duration 5
```

Velocity tests:

```powershell
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode velocity --pattern forward --vx 0.80 --duration 5 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode velocity --pattern forward --vx -0.80 --duration 5 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode velocity --pattern rotate --wz 1.50 --duration 5 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode velocity --pattern rotate --wz -1.50 --duration 5 --monitor
```

Raw PWM tests:

```powershell
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 0 --direction 1 --duty 1.0 --duration 2 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 0 --direction -1 --duty 1.0 --duration 2 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 1 --direction 1 --duty 1.0 --duration 2 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 1 --direction -1 --duty 1.0 --duration 2 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 2 --direction 1 --duty 1.0 --duration 2 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 2 --direction -1 --duty 1.0 --duration 2 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 3 --direction 1 --duty 1.0 --duration 2 --monitor
python upper_computer\python_programs\chassis_bringup_test.py --port COM16 --mode pwmtest --motor 3 --direction -1 --duty 1.0 --duration 2 --monitor
```

## Current Firmware Status

The firmware was rebuilt and flashed successfully after these corrections.

Validation status:

```text
Keil rebuild: 0 Error(s), 0 Warning(s)
ST-Link flash: Download verified successfully
UART feedback: active on COM16 at 115200 baud
```
