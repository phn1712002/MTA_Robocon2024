#include "Arduino.h"
#include "Wire.h"
#include "MPU6050_light.h"
#include "KalmanFilter.h"
#include "I2C_support_float.hpp"

#define ADDRESS_I2C 8

#define ANGLE_KF 0.01
#define BIAS_KF 0.03
#define MEASURE 0.03

#define BAUD_SERIAL 9600
#define PRINT_SERIAL true

// MPU
MPU6050 mpu(Wire);
KalmanFilter kalmanZ(ANGLE_KF, BIAS_KF, MEASURE);
int AngleZ = 0;
bool send = false;

// Error i2c
bool const i2c_debug_rest = true;
int const time_break_i2c = 25000U;

void requestEvent()
{
  float angle_send = AngleZ;
  send = true;
  Wire.write((byte *)&angle_send, sizeof(angle_send));
  send = false;
}
void setup()
{
  Serial.begin(BAUD_SERIAL);
  Wire.begin(ADDRESS_I2C);
  Wire.onRequest(requestEvent);
  Wire.setWireTimeout(time_break_i2c, i2c_debug_rest);
  if (PRINT_SERIAL)
    Serial.println("Config MPU");
  mpu.begin();
  delay(1000);
  mpu.calcOffsets();
  if (PRINT_SERIAL)
    Serial.println("End config MPU");
}

void loop()
{
  if (!send)
  {
    mpu.update();
    AngleZ = int(kalmanZ.update(mpu.getAngleZ(), mpu.getGyroZ()));
    if (PRINT_SERIAL)
      Serial.println(AngleZ);
  }
}
