# Rover II – Motor Control (Subsystem 6)

Owner: Buddy 4 (Lim Zi Ying) · FR-38 to FR-42 · Week 6 Section 6.6 statechart

## Files

| File | What it is |
|---|---|
| `include/motor_pins.h` | Robo Pico pin numbers (same as the team's `config.h`). |
| `include/motor_config.h` | All tuning values (provisional, from Tribunal 1). Tune these on the rover. |
| `include/motor_types.h` | Minimum-bit layouts: command word, heading word, direction byte, result byte, fault bits. Also the data representation table for Week 13. |
| `include/motor_control.h`, `src/motor_control.c` | The Motor Control logic and statechart. No hardware or RTOS calls, so it is unit-tested on a laptop. |
| `include/motor_hw.h`, `src/motor_hw.c` | L298N PWM/direction pins, active brake, claw/arm pins, direction byte on UART TX GP12. |
| `include/motor_task.h`, `src/motor_task.c` | The FreeRTOS Motor Control task and the functions other buddies call. |
| `motor_bench/` | Stand-alone bench firmware: runs only the Motor Control task, driven from a USB serial terminal. |
| `test/` | Laptop unit tests with a simulated rover (`make` to run). |

This repo holds only the motor module. To use it in `integrated-rover-ii`, copy `include/` and `src/` into `firmware/robo_pico/`; the old `motor_driver.c` there can then be retired. It is meant to replace the motor part of `main.c` when the Robo Pico moves to FreeRTOS.

## How it maps to the design

**Statechart 6.6:** `ZERO_GYRO → IDLE → VALIDATE → MANUAL | AUTO_ROTATE | AUTO_DRIVE → RAMP_DOWN → IDLE`. Faults go to `SAFE_STOP`, and only an operator reset (with the fault cleared) returns to `IDLE`. Operator stop goes to `IDLE` with PWM 0 at once. The state is 3 bits.

| Requirement | Where |
|---|---|
| FR-38 inverse kinematics, 2 × L298N | `mc_mix()`, `motor_hw.c` |
| FR-39 manual: forward/back, diagonal, strafe, arc, rotate, park | `MC_MANUAL_DRIVE` (vx, vy, w), `MC_MANUAL_PARK` |
| FR-40 autonomous only forward/back + rotate; manual only in READY | `mc_command()` checks source and READY; `MC_AUTO_DRIVE` has no sideways field |
| FR-41 heading hold + rotate within tolerance | PI `hold_trim()`, `MS_AUTO_ROTATE` (±3°, slows within 15°) |
| FR-42 ramp normal moves; immediate stop on e-stop, operator stop, timeout | 5 %/tick ramp, `hard_stop()`, 300 ms command timeout |
| NFR-03 200 ms halt | STOP wakes the task at once by task notification; PWM off + active brake first |

**Prof feedback / Tribunal 1:**

| Point | Done |
|---|---|
| Motor Control event-driven → fixed period | 20 ms fixed period; the task also wakes early on a STOP notification |
| Heading only 20 Hz → freshness limit | 150 ms limit; the stale flag latches, so the 10-bit time stamp wrapping can't make old heading look fresh |
| One source for motor commands (FR-23 vs 6.6) | Autonomous only from Exploration (`MC_SRC_AUTO`), manual only from Robo Comms in READY, STOP bypasses the queue |
| TBD values → provisional numbers | ±3°, 300 ms timeout, 150 ms freshness, cruise/approach speeds in `motor_config.h` |
| Command timeout vs long segments | Resending the identical word is a refresh; a finished move's late refreshes are ignored |
| Active braking | L298N short brake (IN1 = IN2 = 0, EN on) for 300 ms, no drive current |
| Claw/arm safe at boot | `motor_hw_safe_boot()` is the first call in `main()` |
| PWM slice 5, GP0/GP1, GP28 | See issues below; stdio stays on USB |
| Direction code for Collision Detection | 4-bit "moving toward" mask + 4-bit seq, 1 byte on GP12, sent before motion, on change and every 100 ms |
| Minimum-bit change | Command = 4 bytes (old text command 26 bytes); heading 12 bits; faults in an event group; heading word written in one 32-bit store; the STOP ISR only notifies |

## Interfaces for other buddies

```c
// Exploration (Buddy 1): autonomous moves, refresh every ~100 ms while active
motor_task_send_cmd(mc_pack_auto_rotate(seq, 90));        // rotate to 90 deg
motor_task_send_cmd(mc_pack_auto_drive(seq, 50, 90));     // drive forward, hold 90 deg
motor_task_send_cmd(mc_pack_simple(MC_STOP, MC_SRC_AUTO, seq)); // marker reached
motor_task_set_result_cb(my_cb);   // DONE / HEADING_REACHED / STOPPED / REJECTED + seq

// Robo Comms (Buddy 5)
motor_task_send_cmd(mc_pack_manual_drive(seq, vx, vy, w)); // manual, only in READY
motor_task_set_system_ready(true/false);
motor_task_operator_stop();   // dashboard STOP
motor_task_link_lost();       // FR-08 timeout
motor_task_estop_from_isr();  // UART STOP frame, call from the RX interrupt

// UART heading receiver (Buddy 1 frame)
motor_heading_publish(heading_tenths, frame_seq);  // 0..3599, every frame (20 Hz)
```

Use a new sequence number for each new move; resend the same word to keep it alive.

## Issues found in the existing code (please read)

1. **PWM slice 5 is worse than "same frequency".** GP26 and GP10 are both slice 5 channel A; GP27 and GP11 are both channel B. Pins on the same channel always output the same duty. In the old `motor_driver.c`, driving the arm puts GP10/GP11 into PWM mode, which also sets the back wheels' speed. The new code drives the arm and claw as plain on/off GPIO instead.
2. **Dashboard refresh vs command timeout.** The dashboard (JiaHui branch) repeats a held key every 300 ms, which equals the new 300 ms timeout, so the rover would stutter or stop. Change the dashboard's `HEARTBEAT_MS` to 100 ms (Buddy 5).
3. **Old emergency stop** was 300 ms of 100 % "reverse thrust" with a fixed pattern. All four motors fight each other at stall current, and the pattern does not depend on which way the rover is moving. It is replaced by the L298N short brake.
4. **The old UART packet** (`uart_protocol.h`) still has a magnetometer, float heading and 100 LIDAR points, about 870 bytes per frame. Buddy 1's new frame should carry heading as 12 bits + 4-bit seq and call `motor_heading_publish()`.
5. **Not yet merged into `main.c`**: the team's Robo Pico firmware is still a bare-metal loop. Integration needs the FreeRTOS port of Robo Comms (Buddy 5).

## Before the first run (checklist)

- [ ] Confirm `MC_HEADING_CW_POSITIVE` with Buddy 1: does heading increase when the rover turns clockwise?
- [ ] Check the Robo Pico VBAT jumper for GP28 is open.
- [ ] Wheels off the ground: bench command `t`, each wheel alone; fix `MC_INVERT`.
- [ ] Measure each wheel's deadband (the smallest duty that turns it) → `MC_DEADBAND_PCT`.
- [ ] Measure the % for 0.3 m/s → `MC_AUTO_CRUISE_PCT`.
- [ ] Tune heading hold (A6, A7), then measure drift per minute (A7) and the STOP chain time (NFR-03) with Buddy 1.

## Build

```bash
# laptop unit tests
cd test && make

# bench firmware
export PICO_SDK_PATH=~/pico-sdk FREERTOS_KERNEL_PATH=~/FreeRTOS-Kernel
cd motor_bench && mkdir build && cd build && cmake .. && make   # -> motor_bench.uf2
```

Bench commands are listed at the top of `motor_bench/main.c` (`ready 1`, `m 40 0 0`, `r 90`, `h 89`, `e`, `reset`, `t`, `q`...).
