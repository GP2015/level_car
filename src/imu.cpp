#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// #include <Wire.h>

#include "config.hpp"

Adafruit_MPU6050 mpu;
float roll, pitch, yaw;

void init_imu()
{
    if (!mpu.begin())
    {
        while (1)
        {
            if constexpr (USE_SERIAL_MONITOR)
            {
                Serial.println("Failed to find MPU6050 chip");
            }

            delay(100);
        }
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    roll = 0;
    pitch = 0;
    yaw = 0;
}

void read_orientation()
{
    sensors_event_t e;
    mpu.getEvent(nullptr, &e, nullptr);

    roll += e.gyro.x;
    pitch += e.gyro.y;
    yaw += e.gyro.z;

    if constexpr (PRINT_ROTATION)
    {
        Serial.printf("Rotation (RPY): %f, %f, %f\n", roll, pitch, yaw);
    }
}