
# BLE_WeatherMonitor

A project that uses the BME280 weather sensor, SSD1306 OLED 
panel, and ESP32 development board to read and send weather
data to a secondary device via Bluetooth Low Energy (BLE).
The project also displays the same data on the OLED panel.

# STM32
Development board used for this project is STM32F4 Discovery. 
RTOS was used to handle multiple tasks such as: BME280 data 
retrieval and transmission, OLED data transmission, 
potentiometer OLED dimming, and ESP32 data transmission.

# BME280

Custom driver code for the BME280 weather sensor is found in
Core/Src/BME280.c and Core/Inc/BME280.h

Communication protocol being used to retrieve data is I2C.
Pinout table is located below.

# SSD1306

Custom driver code for the SSD1306 OLED panel is found in 
Core/Src/OLED.c and Core/Inc/OLED.h

Communication protocol being used to display data is SPI.
Pinout table is located below.

# ESP32

BLE state machine source code is from Espressif's GATT
SERVER API documentation page. 

Modified code is found in esp32s_ble_gatts/src/main.c 
and esp32s_ble_gatts/include/gatts_weather_table.h
## Screenshots

The picture shows the wiring between the STM32,
BME280, SSD1306, and ESP32. The project uses an external
power bank to power the project.

![Wiring](https://github.com/Carlos-Pach/BLE_WeatherMonitor/blob/main/20210811_030828.jpeg)

The picture shows a screenshot from the Nordic nRF app.
Under "Environmental Sensing" are the BLE data specifications
for both Temperature and Humidity. 


![nRF_app](https://github.com/Carlos-Pach/BLE_WeatherMonitor/blob/main/Screenshot_20210811-030900_nRF%20Connect.jpeg)


  
## Pinout Reference

#### I2C Pins
```

-----------------------------------------------
|           | SCL       | SDA     | GND       |
| --------- | --------  | ------- | --------- |
| STM32     | PB6       | PB7     | GND       |
| --------- | --------  | ------- | --------- |
| ESP32     | P5        | P4      | GND       |
| --------- | --------  | ------- | --------- |
| BME280    | SCL       | SDA     | GND       |
-----------------------------------------------

```
#### SPI Pins
```

----------------------------------------------------------------------
|           | D0        | D1      | RES       | DC       | CS        |
| --------- | --------  | ------- | --------- | -------- | --------- |
| STM32     | PB13      | PB15    | PB12      | PB14     | PD8       |
| --------- | --------  | ------- | --------- | -------- | --------- |
| SSD1306   | D0        | D1      | RES       | DC       | CS        |
----------------------------------------------------------------------

 ``` 
## Demo video
  [Video](https://youtu.be/3c85SFc8Bkc)