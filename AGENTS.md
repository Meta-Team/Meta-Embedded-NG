# Repository Guidelines

## Project Structure & Module Organization

This is STM32H723 embedded RoboMaster controller firmware built from a CubeMX-style layout. `Core/Inc` and `Core/Src` hold generated startup, peripheral, FreeRTOS, and interrupt code. Keep board support code in `bsp/<peripheral>/`, high-level robot behavior in `application/<feature>/`, and reusable device/control logic in `module/<domain>/`. Vendor code lives under `Drivers/`, `Middlewares/`, and `USB_DEVICE/`; avoid editing it unless regenerating or patching upstream code deliberately. Project metadata and linker configuration are in `Meta-Embedded-NG.ioc` and `STM32H723XG_FLASH.ld`. Module notes are usually stored beside code as `*.md`.

## Build, Test, and Development Commands

**Create the `build/` directory before your first build** — the Makefile's `mkdir` rule does not create parent directories, so a fresh checkout without `build/` fails with `mkdir: cannot create directory 'build/<robot>'`:

```sh
mkdir build      # one-time, unless build/ already exists
```

The firmware is built per robot variant via the `ROBOT` variable (default `sentry`):

- `make -j ROBOT=sentry` / `make -j ROBOT=infantry`: builds `build/<robot>/Meta-Embedded-NG-<robot>.elf`, `.hex`, `.bin`, and the map file with `arm-none-eabi-gcc`. The `-j` flag enables parallel compilation (pass `-j<N>` to cap the job count, e.g. `-j8`). Each variant compiles into its own `build/<robot>/` subdirectory, so switching robots needs no `make clean`.
- `make -j`: same as `make -j ROBOT=sentry` (the default variant).
- `make clean`: removes the current variant's `build/<robot>/` directory (run per `ROBOT`).
- `make -j GCC_PATH=/path/to/gcc-arm-none-eabi/bin`: builds with a specific GNU Arm toolchain without changing `PATH`.

The robot type is selected purely at build time: `make ROBOT=<robot>` passes `-DROBOT_<ROBOT>` and compiles `application/<robot>/` plus its params header. The common message contract and the variant selector live in `application/robot_def.h`; per-robot physical params live in `application/<robot>/<robot>_def.h`. Adding a robot = copy `application/sentry/` to `application/<new>/`, add an `#elif defined(ROBOT_<NEW>)` arm in `robot_def.h`, and add an `else ifeq ($(ROBOT),<new>)` arm in the Makefile's robot-variant block.

There is no repository-level flash target in the Makefile. Use the generated artifacts with the team debugger/programmer workflow.

## Coding Style & Naming Conventions

Use C with the existing firmware style: 4-space indentation, Doxygen file/function comments for public modules, and short inline comments for hardware-specific decisions. Prefer `static` for file-local helpers and state. Follow current names: macros in `UPPER_SNAKE_CASE`, config/data structs ending in `_s`, typedefs ending in `_t`, and init/control APIs like `ChassisInit()` or `DJIMotorControl()`. Keep each feature's `.c`, `.h`, and optional `.md` together in its module directory. Do not hand-edit generated CubeMX blocks unless the change is intentional and documented.

## Testing Guidelines

No first-party host unit test framework is configured. Treat a clean `make` as the minimum verification for every change. For behavior changes, document the hardware smoke test performed, including board role, connected buses, affected motors/sensors, and observed logs or telemetry. Add focused tests or simulation harnesses only when introducing host-testable logic outside HAL-dependent paths.

## Commit & Pull Request Guidelines

Recent history follows Conventional Commits: `feat:`, `fix:`, `chore:`, `doc:`, and `feat!:` for breaking changes. Keep commits scoped and mention tuned parameters or affected modules when relevant, for example `fix: update gimbal PID`. Pull requests should describe the change, list build/hardware validation, link issues when available, and call out CubeMX regeneration, linker changes, protocol changes, or calibration updates.
