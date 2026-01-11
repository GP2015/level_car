#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float roll, pitch, yaw;

void initIMU()
{
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void updateOrientation()
{
    sensors_event_t gyro;
    mpu.getEvent(nullptr, &gyro, nullptr);

    roll = gyro.gyro.x;
    pitch = gyro.gyro.y;
    yaw = gyro.gyro.z;
}