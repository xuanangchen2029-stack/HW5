#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define I2C_BAUD 400000
#define HEARTBEAT_PIN 15

#define MPU6050_ADDR 0x68
#define MPU6050_ALT_ADDR 0x69

#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I     0x75

typedef struct {
    int16_t ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw;
    float ax_g, ay_g, az_g, temp_c, gx_dps, gy_dps, gz_dps;
} mpu6050_data_t;

static uint8_t imu_addr = MPU6050_ADDR;

void drawChar(unsigned char x, unsigned char y, char c) {
    if (c < 0x20 || c > 0x7F) c = '?';
    int index = c - 0x20;
    for (int col = 0; col < 5; col++) {
        unsigned char bits = ASCII[index][col];
        for (int row = 0; row < 8; row++) {
            ssd1306_drawPixel(x + col, y + row, (bits >> row) & 0x01);
        }
    }
    for (int row = 0; row < 8; row++) ssd1306_drawPixel(x + 5, y + row, 0);
}

void drawMessage(unsigned char x, unsigned char y, char *message) {
    unsigned char cursor_x = x;
    while (*message != '\0') {
        drawChar(cursor_x, y, *message);
        cursor_x += 6;
        message++;
    }
}

void drawLine(int x0, int y0, int x1, int y1, unsigned char color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (1) {
        if (x0 >= 0 && x0 < 128 && y0 >= 0 && y0 < 32)
            ssd1306_drawPixel((unsigned char)x0, (unsigned char)y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

uint8_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    uint8_t value = 0;
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, &value, 1, false);
    return value;
}

void i2c_read_burst(uint8_t addr, uint8_t start_reg, uint8_t *buf, size_t len) {
    i2c_write_blocking(I2C_PORT, addr, &start_reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buf, len, false);
}

int16_t combine_bytes(uint8_t high, uint8_t low) {
    return (int16_t)(((uint16_t)high << 8) | low);
}

uint8_t detect_mpu6050_address(void) {
    uint8_t who = i2c_read_reg(MPU6050_ADDR, WHO_AM_I);
    if (who == 0x68 || who == 0x98) return MPU6050_ADDR;
    who = i2c_read_reg(MPU6050_ALT_ADDR, WHO_AM_I);
    if (who == 0x68 || who == 0x98) return MPU6050_ALT_ADDR;
    return 0xFF;
}

void fatal_error_loop(void) {
    gpio_put(HEARTBEAT_PIN, 1);
    while (1) {
        ssd1306_clear();
        drawMessage(0, 0, "IMU NOT FOUND");
        drawMessage(0, 8, "CHECK WIRING");
        drawMessage(0, 16, "POWER RESET");
        drawMessage(0, 24, "NEEDED");
        ssd1306_update();
        sleep_ms(200);
    }
}

void mpu6050_init(uint8_t addr) {
    i2c_write_reg(addr, PWR_MGMT_1, 0x00);
    i2c_write_reg(addr, ACCEL_CONFIG, 0x00);
    i2c_write_reg(addr, GYRO_CONFIG, 0x18);
}

void mpu6050_read_all(uint8_t addr, mpu6050_data_t *d) {
    uint8_t buf[14];
    i2c_read_burst(addr, ACCEL_XOUT_H, buf, 14);
    d->ax_raw   = combine_bytes(buf[0],  buf[1]);
    d->ay_raw   = combine_bytes(buf[2],  buf[3]);
    d->az_raw   = combine_bytes(buf[4],  buf[5]);
    d->temp_raw = combine_bytes(buf[6],  buf[7]);
    d->gx_raw   = combine_bytes(buf[8],  buf[9]);
    d->gy_raw   = combine_bytes(buf[10], buf[11]);
    d->gz_raw   = combine_bytes(buf[12], buf[13]);

    d->ax_g   = d->ax_raw * 0.000061f;
    d->ay_g   = d->ay_raw * 0.000061f;
    d->az_g   = d->az_raw * 0.000061f;
    d->temp_c = d->temp_raw / 340.0f + 36.53f;
    d->gx_dps = d->gx_raw * 0.007630f;
    d->gy_dps = d->gy_raw * 0.007630f;
    d->gz_dps = d->gz_raw * 0.007630f;
}

int main() {
    stdio_init_all();
    gpio_init(HEARTBEAT_PIN);
    gpio_set_dir(HEARTBEAT_PIN, GPIO_OUT);
    gpio_put(HEARTBEAT_PIN, 0);

    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    sleep_ms(100);
    ssd1306_setup();

    imu_addr = detect_mpu6050_address();
    if (imu_addr == 0xFF) fatal_error_loop();

    mpu6050_init(imu_addr);

    absolute_time_t next = make_timeout_time_ms(10);
    bool heartbeat = false;
    mpu6050_data_t imu;

    while (1) {
        heartbeat = !heartbeat;
        gpio_put(HEARTBEAT_PIN, heartbeat);

        mpu6050_read_all(imu_addr, &imu);

        printf("ax=%7d ay=%7d az=%7d temp=%7d gx=%7d gy=%7d gz=%7d | ",
               imu.ax_raw, imu.ay_raw, imu.az_raw, imu.temp_raw,
               imu.gx_raw, imu.gy_raw, imu.gz_raw);
        printf("ax=%+.3fg ay=%+.3fg az=%+.3fg temp=%.2fC\n",
               imu.ax_g, imu.ay_g, imu.az_g, imu.temp_c);

        ssd1306_clear();
        int cx = 64, cy = 16;
        // int ex = cx + (int)(imu.ax_g * 20.0f);
        // int ey = cy - (int)(imu.ay_g * 20.0f);
        int ex = cx - (int)(imu.ax_g * 20.0f);
        int ey = cy + (int)(imu.ay_g * 20.0f);
        if (ex < 0) ex = 0; if (ex > 127) ex = 127;
        if (ey < 0) ey = 0; if (ey > 31) ey = 31;

        drawLine(cx - 3, cy, cx + 3, cy, 1);
        drawLine(cx, cy - 3, cx, cy + 3, 1);
        drawLine(cx, cy, ex, ey, 1);

        char msg1[24], msg2[24];
        sprintf(msg1, "X=%+.2fg", imu.ax_g);
        sprintf(msg2, "Y=%+.2fg", imu.ay_g);
        drawMessage(0, 0, msg1);
        drawMessage(0, 24, msg2);

        ssd1306_update();
        sleep_until(next);
        next = delayed_by_ms(next, 10);
    }
}
