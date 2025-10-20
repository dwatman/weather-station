# weather-station

Weather station with precision sensors, ZigBee connectivity to Home Assistant.

## Wireless Module
STM32WBA5MMG Wireless Module

## Sensors

| Type         | Part           | Accuracy          | Interface   | Voltage       | Current | Response |
| ------------ | -------------- | ----------------- | ----------- | ------------- | ------- | -------- |
| Temperature  | SHT45-AD1F | ± 0.1°C           | I²C         | 1.08 - 3.6 V  | 0.4 uA  | 4 s      |
| Humidity     | SHT45-AD1F | ± 1% RH           | I²C         | 1.08 - 3.6 V  | 0.4 uA  | 4 s      |
| Pressure     | LPS22DF    | ± 0.2 hPa         | I²C/SPI/I3C | 1.7 - 3.6 V   | 9 uA    | fast     |
| CO₂          | SCD41      | ±50.0 ppm + 2.5 % | I²C         | 2.4 - 5.5 V   | 15 mA   | 60 s     |
| VOC + NOx    | SGP41      | ± 15%?            | I²C         | 1.7 - 3.6 V   | 3 mA    | 10 s     |
| Particles    | SPS30      | ± 10%             | I²C/UART    | 4.5 - 5.5 V   | 55 mA   |          |
| Light        | OPT4001    | ± 2%              | I²C         | 1.6 - 3.6 V   | 30 µA   | <0.8 s   |
| Sound        | ICS-43434  | ± ?dB             | I²S         | 1.65 - 3.63 V | 490 µA  |          |
| Acceleration | LIS2HH12   | ± 0.1%            | I²C/SPI     | 1.71 - 3.6 V  | 180 µA  |          |
| IR Rx        | TSOP38436  | N/A               | Digital     | 2.0 - 5.5 V   | 450 µA  |          |

## Indoor Sensor Unit
* ZigBee for communication
* USB powered

### Sensors
* Temperature
* Humidity
* CO2
* VOC + NOx
* Particles
* Light
* IR remote control

## Outdoor Sensor Unit
* ZigBee for communication
* Solar + battery powered
* ~0.5 - 1W solar panel
* 18650 lithium cell for energy storage
* BQ25570 for solar/charge control

### Sensors
* Temperature
* Humidity
* Pressure
* Light (+UV?)
* Sound
* Battery level

## Display Unit
* WiFi for communication
* STM32H???
