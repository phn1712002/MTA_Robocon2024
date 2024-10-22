#include "Arduino.h"
#include "Wire.h"
#include "I2C_support_float.hpp"
#include "BTS7960.h"

#define ADDRESS_I2C 20

#define SERIAL_BUAD 9600

#define PRINT_SEARIAL true

#define EN 8
#define R_PWM 10
#define L_PWM 9

#define MAX_PWM 255
#define MIN_PWM -255

#define MAX_RPM 291
#define MIN_RPM 0

float rpm_request_last = 0;
float rpm_request = 0;
bool turn_right;
bool new_value = false;
BTS7960 motorController(EN, L_PWM, R_PWM);

void receiveEvent(int howMany)
{
  read_i2c_value_float(howMany, rpm_request);
  if (isnan(rpm_request))
    rpm_request = rpm_request_last;
  if (PRINT_SEARIAL)
    Serial.println(rpm_request);
  new_value = true;
}

void setup()
{
  Serial.begin(SERIAL_BUAD);
  Wire.begin(ADDRESS_I2C);
  Wire.onReceive(receiveEvent);
  motorController.Enable();
}
float calc_pwm(float rpm)
{
  float pwm = (rpm * MAX_PWM) / MAX_RPM;
  if (pwm > MAX_PWM)
    return MAX_PWM;
  else
    return pwm;
}

void loop()
{
  int pwr = calc_pwm(abs(rpm_request));
  if (pwr == 0)
  {
    motorController.Stop();
  }
  else
  {
    if (rpm_request > 0)
    {
      motorController.TurnRight(pwr);
    }
    else
    {
      motorController.TurnLeft(pwr);
    }
  }
}
