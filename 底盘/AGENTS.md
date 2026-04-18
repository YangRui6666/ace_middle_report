# Repository Guidelines

## Project Structure & Module Organization
This repository is an STM32F405 + FreeRTOS chassis-control project. Keep application code in `User/` and treat `Core/`, `Drivers/`, and `Middlewares/` as generated or vendor-managed unless a hardware fix requires otherwise.

- `User/app`: FreeRTOS task entry points such as `app_task.c`
- `User/bsp`: board support code for CAN, DBUS, IMU, and other peripherals
- `User/module`: reusable control modules such as `chassis`, `motor_3508`, and `pid`
- `User/config`: project-level configuration and tuning constants
- `cmake/`: ARM toolchain files and STM32CubeMX CMake integration
- Root files: `CMakeLists.txt`, `CMakePresets.json`, `startup_stm32f405xx.s`, `STM32F405XX_FLASH.ld`

## Build, Test, and Development Commands
Use CMake presets from the repository root.

- `cmake --preset Debug`: configure a debug build in `build/Debug`
- `cmake --build --preset Debug`: build the firmware ELF
- `cmake --preset Release`: configure an optimized release build
- `cmake --build --preset Release`: build the release image

When working through Codex CLI, follow `RTK.md` and prefix shell commands with `rtk`, for example `rtk cmake --build --preset Debug`.

## Coding Style & Naming Conventions
Follow `ace代码规范.md`.

- Use 4-space indentation, no tabs
- Use lowercase file and directory names where practical
- Use `snake_case` for functions and variables, and `_t` suffixes for typedef structs
- Put opening braces on their own line
- Keep functions short and module-focused
- Add comments only where logic is non-obvious; prefer Doxygen-style headers for public APIs

## Testing Guidelines
There is no dedicated host-side test suite yet. At minimum:

- build with `Debug` before opening a PR
- validate the main control path on hardware: DT7 input, chassis speed mapping, CAN1 motor feedback, and current output
- document any tuning or hardware assumptions in the PR

If you add tests later, place them outside vendor directories and name them after the module they verify.

## Commit & Pull Request Guidelines
Recent history uses short, direct commit messages describing the change, often in Chinese, for example `加了掉线保护` or `改了串口到usart3`. Keep that pattern: one functional change per commit, written as a concise summary.

PRs should include:

- the purpose and affected modules
- any `.ioc`, CMake, pinout, or protocol changes
- build result and hardware validation performed
- logs, screenshots, or CAN/serial captures when behavior changed
