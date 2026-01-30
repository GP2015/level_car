#include "imu.hpp"

#include "config.hpp"

#if MANUAL_I2C

#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

float roll, pitch, yaw;

void write_to_imu(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission(true);
}

void read_from_imu(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, len, true);

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
}

void init_imu() {
    delay(100);
    write_to_imu(0x6B, 0x00);  // Initialise
    write_to_imu(0x1B, 0x00);  // Set gyro to 250 degrees
    write_to_imu(0x1C, 0x00);  // Set accel to 2g

    roll = 0;
    pitch = 0;
    yaw = 0;

    delay(10);
}

void read_orientation() {
    uint8_t raw[14];
    read_from_imu(0x3B, raw, 14);

    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];

    int16_t gx = (raw[8] << 8) | raw[9];
    int16_t gy = (raw[10] << 8) | raw[11];
    int16_t gz = (raw[12] << 8) | raw[13];

    roll += gx;
    pitch += gy;
    yaw += gz;

#if USE_SERIAL_MONITOR && PRINT_ROTATION
    Serial.printf("Rotation (RPY): %f, %f, %f\n", roll, pitch, yaw);
    delay(500);
#endif
}

#else

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "config.hpp"

Adafruit_MPU6050 mpu;
float roll, pitch, yaw;
long prev_time;

void init_imu() {
    if (!mpu.begin()) {
        while (true) {
#if USE_SERIAL_MONITOR
            Serial.println("Failed to find MPU6050 chip");
#endif

            delay(10);
        };

        return;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    roll = 0;
    pitch = 0;
    yaw = 0;

    prev_time = millis();
}

void read_orientation() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    long current_time = millis();
    long time_dif = current_time - prev_time;

    roll += g.gyro.x * time_dif;
    pitch += g.gyro.y * time_dif;
    yaw += g.gyro.z * time_dif;

    prev_time = current_time;

#if USE_SERIAL_MONITOR && PRINT_ROTATION
    Serial.printf("Rotation (RPY): %f, %f, %f\n", roll, pitch, yaw);
#endif
}

#endif