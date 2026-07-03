# Chassis USART1 Bring-up Tools

This directory contains PC-side helper scripts for the migrated STM32F407 chassis firmware.
The current bring-up path uses USART1 at 115200 baud. In the present test bench the USB-UART is `COM16`.

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

If no PWM waveform is visible on an output pin, first check whether the command requested 100% duty. A 100% compare value can appear as a static high level instead of a square wave.
