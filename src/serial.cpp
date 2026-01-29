#include "serial.hpp"

#include <Arduino.h>

constexpr unsigned long SERIAL_BAUD_RATE = 115200;

void init_serial_monitor() {
    // Begin serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);
    Serial.println("Serial monitor active.");
}