#include <Arduino.h>

constexpr unsigned long SERIAL_BAUD_RATE = 115200;

void setup()
{
    // Begin serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    delay(5000);
    Serial.println("Serial monitor active.");
}

void loop()
{
    aaa
}