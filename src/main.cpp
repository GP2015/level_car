#include <Arduino.h>

#include "actuator.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "serial.hpp"

void setup() {
#if USE_SERIAL_MONITOR
    init_serial_monitor();
#endif

#if HELLO_WORLD
    return;
#endif

#if USE_IMU
    init_imu();
#endif

#if USE_MOTORS
    init_motors();
#endif

#if USE_ACTUATORS
    init_actuators();
#endif
}

void loop() {
#if HELLO_WORLD && USE_SERIAL_MONITOR
    Serial.println("Hello World!");
    return;
#endif

#if USE_MOTORS
    update_motors(true, true, 1, 1);
#endif

#if USE_IMU
    read_orientation();
#endif

#if USE_ACTUATORS
#if USE_IMU
    update_actuators(roll, pitch);
#else
    update_actuators_no_imu();
#endif
#endif
}
