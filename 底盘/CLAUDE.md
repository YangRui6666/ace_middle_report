# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

Configure and build from the repository root using CMake presets:

```bash
cmake --preset Debug                  # configure debug build → build/Debug
cmake --build --preset Debug          # compile firmware ELF
cmake --preset Release                # configure release build
cmake --build --preset Release        # compile release image
```

Prefix with `rtk` when running through the Codex CLI (see `RTK.md`):
```bash
rtk cmake --build --preset Debug
```

There is no host-side test suite. Validation is done on hardware: DT7 input → chassis speed mapping → CAN1 motor feedback → current output.

## Architecture

STM32F405 + HAL + FreeRTOS chassis controller. Three FreeRTOS tasks are created in `main.c` and implemented in `User/app/app_task.c`:

| Task | Period | Responsibility |
|------|--------|----------------|
| `ctrlTask` | 5 ms | Read DBUS remote, update chassis target speed via `chassis_update_from_remote()`, call `imu_update()` |
| `motorTask` | 2 ms | Get wheel RPM targets from chassis, run M3508 speed-loop PID, send CAN current |
| `defaultTask` | — | FreeRTOS idle placeholder |

Data flow: `bsp_dbus` → `chassis` (inverse kinematics) → `motor_3508` (speed PID) → `bsp_can` (CAN1 TX)

### Layer structure under `User/`

- `app/` — task entry points (`app_task.c`) and init sequencing
- `bsp/` — peripheral drivers: `bsp_can` (CAN1 RX/TX with ring buffer), `bsp_dbus` (USART3 DMA, DT7 decode), `bsp_imu` (SPI1, BMI088)
- `module/` — reusable logic: `chassis` (mecanum inverse kinematics, mode FSM), `motor_3508` (speed-loop PID per wheel), `pid` (generic PID), `imu_fusion` + `mahony_ahrs` (attitude estimation)
- `config/app_config.h` — all tunable constants (task periods, PID gains, mechanical parameters, speed limits)

### Key hardware interfaces

- `CAN1` (PB8/PB9) — M3508 motor feedback and current commands
- `USART3` (PD2, DMA) — DT7/DR16 remote receiver (DBUS protocol)
- `SPI1` (PA5–PA7, CS: PC4/PB1) — BMI088 IMU (accel + gyro)

### Chassis modes (`chassis_mode_t`)

`CHASSIS_MODE_STOP` → `CHASSIS_MODE_NORMAL` → `CHASSIS_MODE_GYROSCOPE`

Mode is selected by the DT7 right switch. In `GYROSCOPE` mode the chassis spins while translating (小陀螺). Loss of remote signal for >100 ms triggers `chassis_reset()` and `motor_3508_reset()`.

### Cross-file object access pattern

Modules expose state via pointer-returning getters rather than global variables, following the team convention in `ace代码规范.md`:
```c
const chassis_state_t *chassis_get_state_ptr(void);
```

## Coding Style

Follow `ace代码规范.md`:
- 4-space indentation, no tabs
- `snake_case` for functions and variables; `_t` suffix for typedef structs
- Opening brace on its own line for all blocks
- Doxygen-style headers (`@brief`, `@param`, `@retval`) on all public API functions
- Comments only where logic is non-obvious

## `.ioc` File

`dipan_f405.ioc` is the STM32CubeMX project file. Regenerating code from it will overwrite `Core/` and `Drivers/`. User code lives in `/* USER CODE BEGIN/END */` guards and in `User/` — never place logic outside those guards in CubeMX-managed files.
