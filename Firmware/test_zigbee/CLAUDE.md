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
| Temperature (°C) | Temperature Measurement (0x0402) | CubeMX configured |
| Humidity (%) | Water Content Measurement (0x0405) | CubeMX configured |
| Pressure (hPa) | Pressure Measurement (0x0403) | CubeMX configured |
| Light (lux) | Illuminance Measurement (0x0400) | CubeMX configured |
| CO₂ (ppm) | Custom/manufacturer-specific | Needs code implementation |
| CO (ppm) | Custom/manufacturer-specific | Needs code implementation |
| VOC/NOx (AQI) | Custom/manufacturer-specific | Needs code implementation |
| PM1/PM2.5/PM4/PM10 (µg/m³) | Custom/manufacturer-specific | Needs code implementation |
| Sound (dBm) | Custom/manufacturer-specific | Needs code implementation |

All sensor clusters are on **Endpoint 1** (Device ID: Environmental Sensor).

## Key Source Files
- `STM32_WPAN/App/app_zigbee_endpoint.c` — Endpoint & cluster setup, attribute defines. **Main file for sensor cluster customization.**
- `STM32_WPAN/App/app_zigbee.c` — Stack init, network join, Basic Server config. Contains `APP_ZIGBEE_MFR_NAME`, `APP_ZIGBEE_CHIP_NAME` (used by Z2M for device identification).
- `STM32_WPAN/App/app_zigbee_endpoint.h` / `app_zigbee.h` — Headers
- `Core/Src/main.c` — Peripheral init, main loop
- `test_zigbee.ioc` — CubeMX project file (re-generate with CubeMX if middleware config changes)

## Code Customization Needed (Next Phase)
1. **Basic Server identity** (`app_zigbee.c`): Change `APP_ZIGBEE_MFR_NAME` and `APP_ZIGBEE_CHIP_NAME` to unique values for Zigbee2MQTT device identification. Change `APP_ZIGBEE_BOARD_POWER` from `0x00` to `0x01` (mains power).
2. **Custom clusters** (`app_zigbee_endpoint.c`): Add manufacturer-specific clusters for CO₂, CO, VOC/NOx, PM, and Sound in `APP_ZIGBEE_ConfigEndpoints()` within `USER CODE` blocks.
3. **Attribute updates**: Implement periodic sensor reading → ZCL attribute write logic. Use `ZbZclAttrIntegerWrite()` or similar to update cluster attributes.
4. **Reporting**: Configure ZCL attribute reporting (min/max intervals, reportable change) so the coordinator receives updates.
5. **Zigbee2MQTT external converter**: Required for the coordinator to understand custom clusters and expose all sensors to Home Assistant.

## Coding Style

- **Comments**: Use line comments (`//`) by default. Only use block comments (`/* */`) for large multi-line explanations.
- **Function braces**: Opening brace on the same line as the function signature — `void foo(int x) {` — unless the signature is so long it needs to wrap, in which case put the brace on its own line.
- **Indentation**: CubeMX generated files use 2 spaces for indentation, but the user prefers to use tabs (4-space width) in fresh files. Use the stle appropriate for the file being edited.

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
```
Or use the STM32 VS Code extension build commands.

## Logging
UART logging is enabled via USART1 (ST-LINK VCP, 115200 baud). Log level: INFO. Timestamps and EOL enabled.
Use `LOG_INFO_APP()`, `LOG_ERROR_APP()`, `LOG_WARNING_APP()` macros.
