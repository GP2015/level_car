#include "actuator.hpp"

#include <Arduino.h>

constexpr uint8_t L_DRIVE_PWM_PIN = D12;
constexpr uint8_t L_DRIVE_DIR_PIN = D11;

constexpr uint8_t R_DRIVE_PWM_PIN = D10;
constexpr uint8_t R_DRIVE_DIR_PIN = D9;

constexpr uint8_t TL_LEG_PWM_PIN = D8;
constexpr uint8_t TL_LEG_DIR_PIN = D7;

constexpr uint8_t TR_LEG_PWM_PIN = D4;
constexpr uint8_t TR_LEG_DIR_PIN = D3;

constexpr uint8_t BL_LEG_PWM_PIN = D6;
constexpr uint8_t BL_LEG_DIR_PIN = D5;

constexpr uint8_t BR_LEG_PWM_PIN = D1;
constexpr uint8_t BR_LEG_DIR_PIN = D0;

constexpr uint32_t INITIALISATION_TIME = 10000;
constexpr int INITIAL_DIRECTION = HIGH;

void init_actuators()
{
    pinMode(L_DRIVE_PWM_PIN, OUTPUT);
    pinMode(L_DRIVE_DIR_PIN, OUTPUT);

    pinMode(R_DRIVE_PWM_PIN, OUTPUT);
    pinMode(R_DRIVE_DIR_PIN, OUTPUT);

    pinMode(TL_LEG_PWM_PIN, OUTPUT);
    pinMode(TL_LEG_DIR_PIN, OUTPUT);

    pinMode(TR_LEG_PWM_PIN, OUTPUT);
    pinMode(TR_LEG_DIR_PIN, OUTPUT);

    pinMode(BL_LEG_PWM_PIN, OUTPUT);
    pinMode(BL_LEG_DIR_PIN, OUTPUT);

    pinMode(BR_LEG_PWM_PIN, OUTPUT);
    pinMode(BR_LEG_DIR_PIN, OUTPUT);

    digitalWrite(TL_LEG_PWM_PIN, HIGH);
    digitalWrite(TR_LEG_PWM_PIN, HIGH);
    digitalWrite(BL_LEG_PWM_PIN, HIGH);
    digitalWrite(BR_LEG_PWM_PIN, HIGH);

    digitalWrite(TL_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(TR_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(BL_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(BR_LEG_DIR_PIN, INITIAL_DIRECTION);

    delay(INITIALISATION_TIME);
}

void reset_actuator_pos()
{
    digitalWrite(TL_LEG_PWM_PIN, HIGH);
    digitalWrite(TR_LEG_PWM_PIN, HIGH);
    digitalWrite(BL_LEG_PWM_PIN, HIGH);
    digitalWrite(BR_LEG_PWM_PIN, HIGH);

    digitalWrite(TL_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(TR_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(BL_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(BR_LEG_DIR_PIN, INITIAL_DIRECTION);

    delay(INITIALISATION_TIME);
}