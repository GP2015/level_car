#include <Arduino.h>

#include "config.hpp"
#include "serial.hpp"
#include "motor.hpp"
#include "actuator.hpp"
#include "imu.hpp"

void setup()
{
    if constexpr (USE_SERIAL_MONITOR)
    {
        init_serial_monitor();
    }

    if constexpr (USE_IMU)
    {
        init_imu();
    }

    if constexpr (USE_MOTORS)
    {
        init_motors();
    }

    if constexpr (USE_ACTUATORS)
    {
        init_actuators();
    }
}

void loop()
{
    if constexpr (USE_MOTORS)
    {
        update_motors(true, true, 1, 1);
    }

    if constexpr (USE_IMU)
    {
        read_orientation();
    }
}