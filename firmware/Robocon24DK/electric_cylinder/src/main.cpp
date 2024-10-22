#include "Arduino.h"
#include "Wire.h"
#include "I2C_support_float.hpp"
#include "BTS7960.h"

#define ADDRESS_I2C 10

#define SERIAL_BUAD 9600
#define PRINT_SERIAL true

#define EN 10
#define L_PWM 12
#define R_PWM 11
#define PWM 20
#define BACKWARD 1
#define FORWARD -1

float value_i2c, value_i2c_last;
BTS7960 motor(EN, L_PWM, R_PWM);

void receiveEvent(int howMany)
{
  value_i2c_last = value_i2c;
  read_i2c_value_float(howMany, value_i2c);
  if(isnan(value_i2c)) value_i2c = value_i2c_last;
  if(PRINT_SERIAL) Serial.println(value_i2c);
}

void setup()
{
  Serial.begin(SERIAL_BUAD);
  Wire.begin(ADDRESS_I2C);
  Wire.onReceive(receiveEvent);

  if(PRINT_SERIAL) Serial.println("Config motor");
  motor.Enable();
  motor.Stop();
  if(PRINT_SERIAL) Serial.println("End config motor");
}

void loop()
{
  if (value_i2c == BACKWARD)
  {
    if(PRINT_SERIAL) Serial.println("Backward");
    motor.TurnRight(PWM);
  }
  else if (value_i2c == FORWARD)
  {
    if(PRINT_SERIAL) Serial.println("Forward");
    motor.TurnLeft(PWM);
  }
  else
  {
    if(PRINT_SERIAL) Serial.println("Stop");
    motor.Stop();
  }
}