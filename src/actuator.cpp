#include "actuator.hpp"

#include <Arduino.h>

#include "config.hpp"

void reset_actuator_pos()
{
    digitalWrite(TL_LEG_PWM_PIN, 1);
    digitalWrite(TR_LEG_PWM_PIN, 1);
    digitalWrite(BL_LEG_PWM_PIN, 1);
    digitalWrite(BR_LEG_PWM_PIN, 1);

    digitalWrite(TL_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(TR_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(BL_LEG_DIR_PIN, INITIAL_DIRECTION);
    digitalWrite(BR_LEG_DIR_PIN, INITIAL_DIRECTION);

    delay(ACTUATOR_INIT_TO_END_TIME);

    digitalWrite(TL_LEG_DIR_PIN, 1 - INITIAL_DIRECTION);
    digitalWrite(TR_LEG_DIR_PIN, 1 - INITIAL_DIRECTION);
    digitalWrite(BL_LEG_DIR_PIN, 1 - INITIAL_DIRECTION);
    digitalWrite(BR_LEG_DIR_PIN, 1 - INITIAL_DIRECTION);

    delay(ACTUATOR_INIT_END_TO_MID_TIME);

    digitalWrite(TL_LEG_PWM_PIN, 0);
    digitalWrite(TR_LEG_PWM_PIN, 0);
    digitalWrite(BL_LEG_PWM_PIN, 0);
    digitalWrite(BR_LEG_PWM_PIN, 0);
}

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

    reset_actuator_pos();
}