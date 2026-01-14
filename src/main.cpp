#include <Arduino.h>

#define USE_MOTORS
#define USE_ACTUATORS
#define PRINT_TO_SERIAL_MONITOR

#ifdef USE_ACTUATORS
#include "actuator.hpp"
#endif

// #include "imu.hpp"

#ifdef USE_MOTORS
#include "motor.hpp"
#endif

#ifdef PRINT_TO_SERIAL_MONITOR
#include "serial.hpp"
#endif

void setup()
{
#ifdef PRINT_TO_SERIAL_MONITOR
    init_serial_monitor();
#endif

#ifdef USE_MOTORS
    init_motors();
#endif

#ifdef USE_ACTUATORS
    init_actuators();
#endif
}

void loop()
{
#ifdef USE_MOTORS
    update_motors(true, true, 1, 1);
#endif

    // update_orientation();
}