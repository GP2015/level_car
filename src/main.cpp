
#include <Arduino.h>

#include "imu.hpp"
#include "actuator.hpp"

unsigned long lastTime = 0;

void setup()
{
  // Begin serial communication
  Serial.begin(115200);
  delay(5000);
  Serial.println("Serial monitor active.");
  
  initActuators();
  initIMU();

  lastTime = millis();
  Serial.println("\nTracking orientation...\n");
}

void loop()
{
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  readOrientation(dt);

  // Print orientation every 100ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100)
  {
    Serial.print("Roll: ");
    Serial.print(roll, 2);
    Serial.print("°\tPitch: ");
    Serial.print(pitch, 2);
    Serial.print("°\tYaw: ");
    Serial.print(yaw, 2);
    Serial.println("°");

    lastPrint = millis();
  }

  delay(10);
}

// // I²C address (depends on ADD_SEL jumper on board)
// // According to datasheet, ICM-20689 has default I2C address 0x68 when AD0 = 0
// #define IMU_ADDRESS 0x68

// // ICM-20689 register map (only a subset)
// #define REG_POWER_MANAGEMENT_1    0x6B
// #define REG_SAMPLE_RATE_DIV    0x19
// #define REG_CONFIG        0x1A
// #define REG_GYRO_CONFIG   0x1B
// #define REG_ACCEL_CONFIG  0x1C
// #define REG_ACCEL_XOUT_H  0x3B
// #define REG_GYRO_XOUT_H   0x43

// void imuWriteByte(uint8_t reg, uint8_t data) {
//   Wire.beginTransmission(IMU_ADDRESS);
//   Wire.write(reg);
//   Wire.write(data);
//   Wire.endTransmission();
// }

// uint8_t imuReadByte(uint8_t reg) {
//   Wire.beginTransmission(IMU_ADDRESS);
//   Wire.write(reg);
//   Wire.endTransmission(false);
//   Wire.requestFrom(IMU_ADDRESS, (uint8_t)1);
//   if (Wire.available()) return Wire.read();
//   return 0;
// }

// void imuReadBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
//   Wire.beginTransmission(IMU_ADDRESS);
//   Wire.write(reg);
//   Wire.endTransmission(false);
//   Wire.requestFrom(IMU_ADDRESS, count);
//   uint8_t i = 0;
//   while (Wire.available() && i < count) {
//     dest[i++] = Wire.read();
//   }
// }

// void setup(){
//   Wire.begin();
//
//   // Reset device and wake up
//   imuWriteByte(REG_POWER_MANAGEMENT_1, 0x80);  // reset
//   delay(100);
//   imuWriteByte(REG_POWER_MANAGEMENT_1, 0x01);  // PLL X axis reference
//   delay(100);

//   // Set sample rate to 1 kHz / (1+0) = 1kHz
//   imuWriteByte(REG_SAMPLE_RATE_DIV, 0x00);
//   // Set config – set DLPF (digital low-pass filter) for accel/gyro
//   imuWriteByte(REG_CONFIG, 0x03);      // e.g., DLPF config
//   // Set gyroscope full scale to ±500°/s (bits 4-3 = 01) => 0x08
//   imuWriteByte(REG_GYRO_CONFIG, 0x08);
//   // Set accelerometer full scale to ±4 g (bits 4-3 = 01) => 0x10
//   imuWriteByte(REG_ACCEL_CONFIG, 0x10);

//   Serial.println("IMU initialized");
// }

// void loop() {
//   // Read accel + gyro
//   uint8_t raw[14];
//   imuReadBytes(REG_ACCEL_XOUT_H, 14, raw);

//   int16_t accelX = (int16_t)(raw[0] << 8 | raw[1]);
//   int16_t accelY = (int16_t)(raw[2] << 8 | raw[3]);
//   int16_t accelZ = (int16_t)(raw[4] << 8 | raw[5]);
//   int16_t tempRaw= (int16_t)(raw[6] << 8 | raw[7]);
//   int16_t gyroX  = (int16_t)(raw[8] << 8 | raw[9]);
//   int16_t gyroY  = (int16_t)(raw[10]<< 8 | raw[11]);
//   int16_t gyroZ  = (int16_t)(raw[12]<< 8 | raw[13]);

//   // Convert to physical units
//   const float accelScale = 4.0 / 32768.0;   // ±4 g
//   const float gyroScale  = 500.0 / 32768.0; // ±500°/s

//   float ax = accelX * accelScale;
//   float ay = accelY * accelScale;
//   float az = accelZ * accelScale;
//   float gx = gyroX  * gyroScale * DEG_TO_RAD; // convert to rad/s
//   float gy = gyroY  * gyroScale * DEG_TO_RAD;
//   float gz = gyroZ  * gyroScale * DEG_TO_RAD;

//   // Compute roll & pitch from accel
//   float rollAcc  = atan2(ay, az);
//   float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az));

//   static float roll = 0, pitch = 0;
//   static uint32_t lastTime = millis();
//   uint32_t now = millis();
//   float dt = (now - lastTime) / 1000.0;
//   lastTime = now;

//   // complementary filter
//   const float alpha = 0.98;
//   roll  = alpha * (roll  + gx * dt) + (1.0 - alpha) * rollAcc;
//   pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * pitchAcc;

//   // Yaw – naive integration only (will drift)
//   static float yaw = 0;
//   yaw += gz * dt;

//   // Convert to degrees for display
//   float rollDeg  = roll  * 180.0 / PI;
//   float pitchDeg = pitch * 180.0 / PI;
//   float yawDeg   = yaw   * 180.0 / PI;

//   Serial.print("ROLL: ");  Serial.print(rollDeg,2);
//   Serial.print("  PITCH: "); Serial.print(pitchDeg,2);
//   Serial.print("  YAW: ");   Serial.println(yawDeg,2);

//   delay(50);
// }
