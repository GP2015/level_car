#include "motor.hpp"

#include <Arduino.h>

constexpr uint8_t L_DRIVE_PWM_PIN = D12;
constexpr uint8_t L_DRIVE_DIR_PIN = D11;

constexpr uint8_t R_DRIVE_PWM_PIN = D10;
constexpr uint8_t R_DRIVE_DIR_PIN = D9;

constexpr float MAX_MOTOR_POWER = 255;

void init_motors()
{
    pinMode(L_DRIVE_PWM_PIN, OUTPUT);
    pinMode(L_DRIVE_DIR_PIN, OUTPUT);

    pinMode(R_DRIVE_PWM_PIN, OUTPUT);
    pinMode(R_DRIVE_DIR_PIN, OUTPUT);
}

void update_motors(bool l_dir, bool r_dir, float l_pow, float r_pow)
{
    digitalWrite(L_DRIVE_DIR_PIN, l_dir ? 1 : 0);
    digitalWrite(R_DRIVE_DIR_PIN, r_dir ? 1 : 0);

    analogWrite(L_DRIVE_PWM_PIN, l_pow * MAX_MOTOR_POWER);
    analogWrite(R_DRIVE_PWM_PIN, r_pow * MAX_MOTOR_POWER);
}