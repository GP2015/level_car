#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "config.hpp"

Adafruit_MPU6050 mpu;
float roll, pitch, yaw;

bool imu_ok = true;

void init_imu() {
    if (!mpu.begin()) {
#if USE_SERIAL_MONITOR
        Serial.println("Failed to find MPU6050 chip");
#endif
        imu_ok = false;
        delay(500);
        return;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    roll = 0;
    pitch = 0;
    yaw = 0;
}

void read_orientation() {
    if (!imu_ok) {
#if USE_SERIAL_MONITOR
        Serial.println("The IMU has failed to initialise.");
#endif

        return;
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    roll += g.gyro.x;
    pitch += g.gyro.y;
    yaw += g.gyro.z;

#if USE_SERIAL_MONITOR && PRINT_ROTATION
    Serial.printf("Rotation (RPY): %f, %f, %f\n", roll, pitch, yaw);
    delay(500);
#endif
}