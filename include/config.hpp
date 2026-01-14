#pragma once

constexpr bool USE_MOTORS = false;
constexpr bool USE_ACTUATORS = true;
constexpr bool USE_IMU = false;

constexpr bool USE_SERIAL_MONITOR = false;
constexpr bool PRINT_ROTATION = false;

constexpr uint32_t ACTUATOR_INIT_TO_END_TIME = 8000;
constexpr uint32_t ACTUATOR_INIT_END_TO_MID_TIME = 3750;
constexpr int INITIAL_DIRECTION = 1;

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