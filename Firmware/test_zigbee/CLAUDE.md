# Zigbee Environmental Sensor — End Device Firmware

## Project Overview
Zigbee End Device firmware for NUCLEO-WBA55CG that transmits environmental sensor data to a Zigbee coordinator (Zigbee2MQTT + Home Assistant).

- **Target Board**: NUCLEO-WBA55CG (STM32WBA55CGU6)
- **Firmware Package**: STM32CubeWBA 1.9.0
- **Toolchain**: CMake (for VS Code + STM32 extension)
- **Zigbee Stack**: R23, Full Function Device, Centralized End Device (non-sleepy)
- **Channel**: 11
- **No RTOS** — uses STM32_WPAN sequencer (`UTIL_SEQ`)
- **LL drivers** preferred over HAL where possible

## Sensor Data Types
| Sensor | ZCL Cluster | Status |
|---|---|---|
| Temperature (°C) | Temperature Measurement (0x0402) | Implemented (synthetic data) |
| Humidity (%) | Water Content Measurement (0x0405) | Implemented (synthetic data) |
| Pressure (hPa) | Pressure Measurement (0x0403) | Implemented (synthetic data) |
| Light (lux) | Illuminance Measurement (0x0400) | Implemented (synthetic data) |
| CO₂ (ppm) | Mfr-specific 0xFC00 (mfr 0x1002) | Implemented (synthetic data) |
| CO (ppm) | Mfr-specific 0xFC01 (mfr 0x1002) | Implemented (synthetic data) |
| VOC Index / NOx Index | Mfr-specific 0xFC02 (mfr 0x1002) | Implemented (synthetic data) |
| PM1/PM2.5/PM4/PM10 (µg/m³) | Mfr-specific 0xFC03 (mfr 0x1002) | Implemented (synthetic data) |

All sensor clusters are on **Endpoint 1** (Device ID: Environmental Sensor).

## Key Source Files
- `STM32_WPAN/App/app_zigbee_endpoint.c` — Endpoint & cluster setup, attribute defines, periodic sensor update task. **Main file for sensor cluster customization.**
- `STM32_WPAN/App/app_zigbee.c` — Stack init, network join, Basic Server config. Contains `APP_ZIGBEE_MFR_NAME`, `APP_ZIGBEE_CHIP_NAME` (used by Z2M for device identification). **Note:** identity defines are outside USER CODE blocks — re-apply after CubeMX regen.
- `STM32_WPAN/App/app_zigbee_endpoint.h` / `app_zigbee.h` — Headers
- `Core/Src/main.c` — Peripheral init, main loop
- `Core/Src/sensor_data.c` / `Core/Inc/sensor_data.h` — Synthetic sensor data placeholders (replace with real drivers later)
- `test_zigbee.ioc` — CubeMX project file (re-generate with CubeMX if middleware config changes)

## Completed Customizations
1. **Basic Server identity** (`app_zigbee.c`): MFR_NAME="DW", CHIP_NAME="EnvSensor", BOARD_POWER=0x01 (mains).
2. **Custom clusters** (`app_zigbee_endpoint.c`): 4 manufacturer-specific clusters (0xFC00-0xFC03) with mfr code 0x1002, allocated in `APP_ZIGBEE_ConfigEndpoints2` USER CODE block.
3. **Attribute updates**: Periodic sensor update task (30s interval) using UTIL_TIMER + UTIL_SEQ (CFG_TASK_ZIGBEE_APP1). Writes all 12 sensor values via `ZbZclAttrIntegerWrite()`.
4. **Reporting**: Default reporting configured on all custom attributes (min=10s, max=300s).
5. **Synthetic data**: `sensor_data.c` provides slowly-varying placeholder values. Replace with real sensor drivers when hardware is available.

## Architecture Notes
- **CubeMX-generated code** lives in `Core/`, `Drivers/`, `Middlewares/`, `System/`, `STM32_WPAN/`, and `cmake/stm32cubemx/`. These are regenerated from `test_zigbee.ioc` — only modify code within `USER CODE` blocks.
- **User-written code** goes in `Core/Src/` (added manually to top-level `CMakeLists.txt`) or within USER CODE blocks of generated files.
- **Zigbee stack flow**: `app_zigbee.c` initializes the stack and joins the network → `app_zigbee_endpoint.c` configures Endpoint 1 with all sensor clusters and attributes → a periodic timer task reads from `sensor_data.c` and writes attribute values → ZCL reporting sends updates to the coordinator.
- **Precompiled libraries**: The Zigbee stack and MAC layer are ST-provided `.a` files under `Middlewares/`, not source code. They have circular link dependencies (handled by `--start-group`/`--end-group` in CMakeLists.txt).

## Remaining Work
1. **Zigbee2MQTT external converter**: Required for the coordinator to understand custom clusters and expose all sensors to Home Assistant.
2. **Real sensor drivers**: Replace `sensor_data.c` placeholder functions with actual I2C/SPI/UART sensor communication.

## Coding Style

- **Comments**: Use line comments (`//`) by default. Only use block comments (`/* */`) for large multi-line explanations.
- **Function braces**: Opening brace on the same line as the function signature — `void foo(int x) {` — unless the signature is so long it needs to wrap, in which case put the brace on its own line.
- **Indentation**: CubeMX generated files use 2 spaces for indentation, but the user prefers to use tabs (4-space width) in fresh files. Use the style appropriate for the file being edited.

## Adding New Source Files

User-created `.c` files are **not** picked up automatically. After creating a new file in `Core/Src/`, add it to the `target_sources` block in the project's top-level `CMakeLists.txt`:

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/your_new_file.c
)
```

CubeMX-generated files are handled separately by `cmake/stm32cubemx/` and do not need to be added manually.

## CubeMX Re-generation
User code placed within `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks is preserved when re-generating from the .ioc file. Keep all custom code within these blocks.

## Build
```bash
# Configure (from project root)
cmake --preset Debug
# Build
cmake --build build/Debug

# Release build
cmake --preset Release
cmake --build build/Release
```
Or use the STM32 VS Code extension build commands.

Build uses Ninja generator with `cmake/gcc-arm-none-eabi.cmake` toolchain file (ARM cross-compilation). There are no unit tests or linting configured for this project.

## RAM Layout
The linker script (`STM32WBA55xx_FLASH.ld`) merges SRAM1 and SRAM2 into a single 128 KB region (0x20000000–0x2001FFFF). This is safe because the device is mains-powered and low power modes are disabled. **If low power modes (stop2/standby) are ever enabled, SRAM2 retention must be verified** — SRAM2 may not be retained in all sleep states.

## Linker Workarounds (top-level CMakeLists.txt)
The top-level `CMakeLists.txt` is **not** regenerated by CubeMX, so it is the right place for linker fixes:
- **`--start-group` / `--end-group`**: Wraps all libraries to resolve circular dependencies between ST precompiled `.a` files (zigbee stack ↔ MAC layer). Without this, link order causes undefined-reference errors.
- **`--no-warn-execstack`**: Suppresses a harmless warning from ST's toolchain `crtn.o` missing `.note.GNU-stack`.

## Logging
UART logging is enabled via USART1 (ST-LINK VCP, 115200 baud). Log level: INFO. Timestamps and EOL enabled.
Use `LOG_INFO_APP()`, `LOG_ERROR_APP()`, `LOG_WARNING_APP()` macros.

## Broader Project Context
This firmware is part of a weather-station project (`/home/daniel/git/weather-station/`) that includes indoor and outdoor sensor units (Zigbee) and a display unit (WiFi). The production hardware uses an STM32WBA5MMG module (not the NUCLEO dev board). Real sensors include SHT45, LPS22DF, SCD41, SGP41, SPS30, OPT4001 — all currently represented by synthetic data in `sensor_data.c`.
