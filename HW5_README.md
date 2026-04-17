# HW5 - MPU6050 + SSD1306 OLED

## Files for HW5 folder
- HW5_main.c
- ssd1306.c
- ssd1306.h
- font.h
- CMakeLists.txt

## Wiring
Use the same I2C0 bus for both devices:
- Pico 3V3(OUT) -> MPU6050 VCC
- Pico GND -> MPU6050 GND
- Pico GP0 / I2C0 SDA -> MPU6050 SDA
- Pico GP1 / I2C0 SCL -> MPU6050 SCL
- Pico 3V3(OUT) -> OLED VDD/VCC
- Pico GND -> OLED GND
- Pico GP0 / I2C0 SDA -> OLED SDA
- Pico GP1 / I2C0 SCL -> OLED SCL
- Pico GP15 -> heartbeat LED -> 330 ohm -> GND

## What the code does
- Initializes I2C and the SSD1306 display
- Detects MPU6050 at 0x68, and also tries alternate address 0x69
- Accepts WHO_AM_I = 0x68 or 0x98
- Initializes MPU6050:
  - PWR_MGMT_1 = 0x00
  - ACCEL_CONFIG = 0x00 (+/-2g)
  - GYRO_CONFIG = 0x18 (+/-2000 dps)
- Burst reads 14 bytes from ACCEL_XOUT_H
- Recombines bytes into signed 16-bit values
- Converts accel to g, gyro to dps, temperature to deg C
- Prints data to the computer at 100 Hz
- Draws a line on the OLED from the center using X/Y acceleration
