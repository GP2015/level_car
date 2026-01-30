#include "actuator.hpp"

#include <Arduino.h>

#include "config.hpp"

void reset_actuator_pos() {
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

void init_actuators() {
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

void update_actuators(float pitch, float roll) {
    if (pitch < 0) {
        digitalWrite(TL_LEG_DIR_PIN, 1);
        digitalWrite(BR_LEG_DIR_PIN, 0);
        digitalWrite(TL_LEG_PWM_PIN, 1);
        digitalWrite(BR_LEG_PWM_PIN, 1);
    } else if (pitch > 0) {
        digitalWrite(TR_LEG_DIR_PIN, 1);
        digitalWrite(BL_LEG_DIR_PIN, 0);
        digitalWrite(TR_LEG_PWM_PIN, 1);
        digitalWrite(BL_LEG_PWM_PIN, 1);
    }

    if (roll > 0) {
        digitalWrite(BL_LEG_DIR_PIN, 1);
        digitalWrite(TR_LEG_DIR_PIN, 0);
        digitalWrite(BL_LEG_PWM_PIN, 1);
        digitalWrite(TR_LEG_PWM_PIN, 1);
    } else if (pitch > 0) {
        digitalWrite(BR_LEG_DIR_PIN, 1);
        digitalWrite(TL_LEG_DIR_PIN, 0);
        digitalWrite(BR_LEG_PWM_PIN, 1);
        digitalWrite(TL_LEG_PWM_PIN, 1);
    }
}

#define DELAY_CONSTANT 500

enum Leg {
    TOP_LEFT,
    TOP_RIGHT,
    BOTTOM_LEFT,
    BOTTOM_RIGHT
};

void drive_leg(Leg leg, uint8_t dir) {
    switch (leg) {
        case TOP_LEFT:
            digitalWrite(TL_LEG_PWM_PIN, 1);
            digitalWrite(TL_LEG_DIR_PIN, dir ? 1 : 0);
            break;
        case TOP_RIGHT:
            digitalWrite(TR_LEG_PWM_PIN, 1);
            digitalWrite(TR_LEG_DIR_PIN, dir ? 1 : 0);
            break;
        case BOTTOM_LEFT:
            digitalWrite(BL_LEG_PWM_PIN, 1);
            digitalWrite(BL_LEG_DIR_PIN, dir ? 1 : 0);
            break;
        case BOTTOM_RIGHT:
            digitalWrite(BR_LEG_PWM_PIN, 1);
            digitalWrite(BR_LEG_DIR_PIN, dir ? 1 : 0);
            break;
    }
}

void stop_leg(Leg leg) {
    switch (leg) {
        case TOP_LEFT:
            digitalWrite(TL_LEG_PWM_PIN, 0);
            break;
        case TOP_RIGHT:
            digitalWrite(TR_LEG_PWM_PIN, 0);
            break;
        case BOTTOM_LEFT:
            digitalWrite(BL_LEG_PWM_PIN, 0);
            break;
        case BOTTOM_RIGHT:
            digitalWrite(BR_LEG_PWM_PIN, 0);
            break;
    }
}

void update_actuators_no_imu() {
    drive_leg(TOP_LEFT, 1);
    stop_leg(BOTTOM_RIGHT);

    delay(DELAY_CONSTANT);

    drive_leg(TOP_RIGHT, 1);
    stop_leg(BOTTOM_LEFT);

    delay(DELAY_CONSTANT);

    drive_leg(BOTTOM_RIGHT, 1);
    stop_leg(TOP_LEFT);

    delay(DELAY_CONSTANT);

    drive_leg(BOTTOM_LEFT, 1);
    stop_leg(TOP_RIGHT);

    delay(DELAY_CONSTANT);

    drive_leg(TOP_LEFT, 0);
    stop_leg(BOTTOM_RIGHT);

    delay(DELAY_CONSTANT);

    drive_leg(TOP_RIGHT, 0);
    stop_leg(BOTTOM_LEFT);

    delay(DELAY_CONSTANT);

    drive_leg(BOTTOM_RIGHT, 0);
    stop_leg(TOP_LEFT);

    delay(DELAY_CONSTANT);

    drive_leg(BOTTOM_LEFT, 0);
    stop_leg(TOP_RIGHT);

    delay(DELAY_CONSTANT);
}