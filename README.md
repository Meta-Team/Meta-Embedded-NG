# Meta-Embedded-NG

> 面向达妙 **DM-MC02（STM32H723VGT6）** 控制板的 RoboMaster 机器人电控固件。
> RoboMaster robot control firmware for the DaMiao **DM-MC02 (STM32H723VGT6)** board.

[![MCU](https://img.shields.io/badge/MCU-STM32H723VGT6-blue)](#)
[![Board](https://img.shields.io/badge/Board-DM--MC02-orange)](#)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)](#)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](#license--致谢--license--acknowledgements)

**[简体中文](#简体中文) | [English](#english)**

---

## 简体中文

### 项目简介

**Meta-Embedded-NG** 是一套用于 RoboMaster 比赛机器人的嵌入式电控固件。它在湖南大学跃鹿战队开源框架 [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework) 的基础上做了大量修改，将原本面向 STM32F407（大疆 C 板）的代码移植并适配到了基于 **STM32H723** 的 **大疆 DM-MC02** 控制板。

项目沿用了原框架优秀的设计思想：

- **三层分层架构** —— `bsp`（板级支持）→ `module`（可复用算法与设备驱动）→ `application`（机器人行为）。
- **消息中心（Message Center）发布-订阅** —— 各应用之间不直接相互包含，而是通过话题（topic）解耦通信。
- **离线守护（Daemon）** —— 统一监控各模块心跳，掉线时触发回调以进入安全状态。

### 硬件平台

| 项目 | 说明 |
| --- | --- |
| 主控 MCU | STM32H723VGT6（Cortex-M7，最高 550 MHz，1 MB Flash） |
| 控制板 | 大疆 DM-MC02 |
| 板载 IMU | BMI088（SPI 接口，含温度补偿与在线标定） |
| 链接脚本 | `STM32H723XG_FLASH.ld` |
| RTOS | FreeRTOS |

### 与 basic_framework 的差异

本项目是对 `basic_framework` 的**深度移植与裁剪**，主要区别如下：

| 方面 | basic_framework | Meta-Embedded-NG |
| --- | --- | --- |
| 主控 / 控制板 | STM32F407（大疆 C 板） | STM32H723（DM-MC02） |
| 模块目录名 | `modules/` | `module/` |
| 电机驱动 | DJI / DM / HT / LK / 舵机 / 步进 | DJI / DM / 舵机 / 步进 / **XM（新增）**，移除 HT、LK |
| 传感器/外设模块 | 含 IST8310 磁力计、OLED、蓝牙、TFminiPlus 激光雷达 | 已移除上述模块 |
| 裁判系统 | 原版 | 修复了裁判系统协议与任务 |

核心三层架构、消息中心设计与代码规范保持不变。

### 目录结构

```
Meta-Embedded-NG/
├── application/      # 机器人行为：底盘、云台、发射、指令分发、哨兵
├── module/           # 可复用算法与设备驱动（电机、IMU、消息中心等）
├── bsp/              # 板级支持包，封装 STM32H7 片上外设
├── Core/             # CubeMX 生成的外设/FreeRTOS/中断代码（Inc + Src）
├── Drivers/          # STM32H7xx HAL 与 CMSIS（厂商代码）
├── Middlewares/      # FreeRTOS、USB 设备库、CMSIS-DSP、SEGGER RTT
├── USB_DEVICE/       # USB 设备中间件（App + Target）
├── Makefile          # arm-none-eabi-gcc 构建脚本
├── Meta-Embedded-NG.ioc   # CubeMX 工程文件
└── STM32H723XG_FLASH.ld   # 链接脚本
```

### 软件架构

- **application/**：`chassis`（底盘运动学/功率限制）、`gimbal`（云台姿态控制）、`shoot`（摩擦轮 + 拨弹）、`cmd`（指令分发，消费遥控/键鼠输入并发布控制量）、`sentry`（哨兵专用行为）。机器人类型、双板模式与标定偏置在 `application/robot_def.h` 中配置。
- **module/**：电机驱动（`motor/`：DJI、DM、XM、舵机、步进）、`algorithm`（PID、卡尔曼滤波、LQR、EKF 姿态解算、CRC）、`message_center`、`daemon`、`BMI088`、`imu`、`encoder`、`can_comm`（多板 CAN 通信）、`super_cap`（超级电容）、`referee`（裁判系统/UI）、`remote`（DT7 遥控）、`master_machine`、`standard_cmd`、`unicomm`、`alarm`。
- **bsp/**：`can`、`usart`、`spi`、`iic`、`pwm`、`adc`、`gpio`、`log`、`usb`、`flash`、`dwt`。
- **FreeRTOS 任务**：INS 姿态解算（约 1 kHz）、电机控制、系统监控、机器人主逻辑等并行运行。

各子模块旁通常附带 `*.md` 说明文档（如 `module/message_center/message_center.md`、`module/motor/DJImotor/dji_motor.md`、`module/BMI088/bmi088.md`），开发前建议先阅读。

### 环境与构建

依赖 **GNU Arm Embedded 工具链**（`arm-none-eabi-gcc`）。

```bash
# 构建（生成 build/Meta-Embedded-NG.elf / .hex / .bin / .map）
make

# 清理 build/ 目录
make clean

# 指定工具链路径而不修改 PATH
make GCC_PATH=/path/to/gcc-arm-none-eabi/bin
```

> 仓库未提供烧录目标（flash target）。请使用团队的调试器/烧录器（如 OpenOCD + CMSIS-DAP、Ozone 等）将生成的 `.hex`/`.bin` 烧录到板子。

### 配置说明

- 机器人类型、双板（底盘板/云台板）模式、云台编码器偏置等通过 `application/robot_def.h` 中的宏配置。
- BMI088 支持上电在线标定，标定相关参数见 `module/BMI088/bmi088.md`。

### 代码规范与贡献

详见 [`AGENTS.md`](AGENTS.md)。要点：

- 遵循 Conventional Commits：`feat:`、`fix:`、`chore:`、`doc:`、`feat!:`（破坏性变更）。
- C 代码 4 空格缩进；宏 `UPPER_SNAKE_CASE`，配置结构体以 `_s` 结尾，typedef 以 `_t` 结尾，初始化/控制 API 形如 `ChassisInit()`、`DJIMotorControl()`。
- CAN/串口等协议结构体使用 `#pragma pack(1)`。
- 每次改动至少保证 `make` 通过；行为变更请记录硬件验证情况。
- 不要手动修改 CubeMX 生成块，除非是有意且记录在案的改动。

### License & 致谢

本项目以 **MIT License** 发布，Copyright (c) 2025 Meta-Team，详见 [`LICENSE`](LICENSE)。

衷心感谢 **湖南大学 RoboMaster 跃鹿战队** 开源的 [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework)（MIT，Copyright (c) 2022 NeoZng / Even），本项目在其基础上开发。

---

## English

### Overview

**Meta-Embedded-NG** is embedded control firmware for RoboMaster competition robots. It is heavily modified from the Hunan University YueLu Team's open-source [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework), porting and adapting code originally targeting the STM32F407 (DJI C-board) to the **STM32H723**-based **DJI DM-MC02** control board.

The project keeps the original framework's core design ideas:

- **Three-layer architecture** — `bsp` (board support) → `module` (reusable algorithms & device drivers) → `application` (robot behavior).
- **Message Center pub-sub** — applications never include each other directly; they communicate through decoupled topics.
- **Daemon offline watchdog** — monitors module heartbeats and triggers a callback to enter a safe state on timeout.

### Hardware

| Item | Detail |
| --- | --- |
| MCU | STM32H723VGT6 (Cortex-M7, up to 550 MHz, 1 MB Flash) |
| Board | DJI DM-MC02 |
| Onboard IMU | BMI088 (SPI, with temperature compensation & online calibration) |
| Linker script | `STM32H723XG_FLASH.ld` |
| RTOS | FreeRTOS |

### Differences from basic_framework

This project is a **deep port and trim** of `basic_framework`. Key differences:

| Aspect | basic_framework | Meta-Embedded-NG |
| --- | --- | --- |
| MCU / board | STM32F407 (DJI C-board) | STM32H723 (DM-MC02) |
| Modules folder | `modules/` | `module/` |
| Motor drivers | DJI / DM / HT / LK / servo / stepper | DJI / DM / servo / stepper / **XM (new)**; HT & LK removed |
| Sensor/peripheral modules | Includes IST8310 magnetometer, OLED, Bluetooth, TFminiPlus LiDAR | These modules removed |
| Referee system | Original | Referee protocol & task fixed |

The core three-layer architecture, Message Center design, and coding conventions are unchanged.

### Project Structure

```
Meta-Embedded-NG/
├── application/      # Robot behavior: chassis, gimbal, shoot, cmd dispatch, sentry
├── module/           # Reusable algorithms & device drivers (motors, IMU, message center...)
├── bsp/              # Board support package wrapping STM32H7 on-chip peripherals
├── Core/             # CubeMX-generated peripheral/FreeRTOS/interrupt code (Inc + Src)
├── Drivers/          # STM32H7xx HAL & CMSIS (vendor code)
├── Middlewares/      # FreeRTOS, USB device library, CMSIS-DSP, SEGGER RTT
├── USB_DEVICE/       # USB device middleware (App + Target)
├── Makefile          # arm-none-eabi-gcc build script
├── Meta-Embedded-NG.ioc   # CubeMX project file
└── STM32H723XG_FLASH.ld   # Linker script
```

### Architecture

- **application/**: `chassis` (kinematics / power limiting), `gimbal` (attitude control), `shoot` (friction wheels + loader), `cmd` (command dispatch — consumes remote/keyboard input and publishes control setpoints), `sentry` (sentry-specific behavior). Robot type, dual-board mode, and calibration offsets are configured in `application/robot_def.h`.
- **module/**: motor drivers (`motor/`: DJI, DM, XM, servo, stepper), `algorithm` (PID, Kalman filter, LQR, EKF attitude solver, CRC), `message_center`, `daemon`, `BMI088`, `imu`, `encoder`, `can_comm` (multi-board CAN), `super_cap`, `referee` (referee system / UI), `remote` (DT7 receiver), `master_machine`, `standard_cmd`, `unicomm`, `alarm`.
- **bsp/**: `can`, `usart`, `spi`, `iic`, `pwm`, `adc`, `gpio`, `log`, `usb`, `flash`, `dwt`.
- **FreeRTOS tasks**: INS attitude estimation (~1 kHz), motor control, system monitoring, and the main robot logic run concurrently.

Most submodules ship with a companion `*.md` doc (e.g. `module/message_center/message_center.md`, `module/motor/DJImotor/dji_motor.md`, `module/BMI088/bmi088.md`) — read them before diving in.

### Build

Requires the **GNU Arm Embedded toolchain** (`arm-none-eabi-gcc`).

```bash
# Build (produces build/Meta-Embedded-NG.elf / .hex / .bin / .map)
make

# Clean the build/ directory
make clean

# Use a specific toolchain without changing PATH
make GCC_PATH=/path/to/gcc-arm-none-eabi/bin
```

> There is no flash target in the Makefile. Use your team's debugger/programmer (e.g. OpenOCD + CMSIS-DAP, Ozone) to flash the generated `.hex`/`.bin` to the board.

### Configuration

- Robot type, dual-board (chassis-board / gimbal-board) mode, and gimbal encoder offsets are set via macros in `application/robot_def.h`.
- BMI088 supports power-on online calibration; see `module/BMI088/bmi088.md` for parameters.

### Conventions & Contributing

See [`AGENTS.md`](AGENTS.md). Highlights:

- Follow Conventional Commits: `feat:`, `fix:`, `chore:`, `doc:`, `feat!:` (breaking change).
- 4-space indentation; macros `UPPER_SNAKE_CASE`, config structs end in `_s`, typedefs end in `_t`, init/control APIs like `ChassisInit()` / `DJIMotorControl()`.
- Use `#pragma pack(1)` for CAN/serial protocol structs.
- A clean `make` is the minimum verification for every change; document hardware validation for behavior changes.
- Do not hand-edit generated CubeMX blocks unless the change is intentional and documented.

### License & Acknowledgements

Released under the **MIT License**, Copyright (c) 2025 Meta-Team. See [`LICENSE`](LICENSE).

Sincere thanks to the **Hunan University RoboMaster YueLu Team** for the open-source [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework) (MIT, Copyright (c) 2022 NeoZng / Even), on which this project is built.
