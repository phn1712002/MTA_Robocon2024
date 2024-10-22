#include "Arduino.h"
#include "Wire.h"
#include "I2C_support_float.hpp"
#include "BTS7960.h"
#include "Switch_support.hpp"
#include "Relay_support.hpp"

#define PRINT_SERIAL true

#define ADDRESS_I2C 11
#define SERIAL_BUAD 9600

#define EN 10
#define L_PWM 12
#define R_PWM 11
#define PWM 255

#define IN_RL 9
#define IN_SW_F 3
#define IN_SW_B 2

#define NOT_WORKING -1
#define KEY_FIRE 1

float value_i2c = NOT_WORKING, value_i2c_last = 0;
BTS7960 motor(EN, L_PWM, R_PWM);
Switch sw_f(IN_SW_F);
Switch sw_b(IN_SW_B);
Relay12V rl(IN_RL);

void receiveEvent(int howMany)
{
  if (value_i2c == NOT_WORKING)
  {
    value_i2c_last = value_i2c;
    read_i2c_value_float(howMany, value_i2c);
    if(isnan(value_i2c)) value_i2c = value_i2c_last;
    if (value_i2c != KEY_FIRE)
    {
      value_i2c = NOT_WORKING;
    }
    if (PRINT_SERIAL)
      Serial.println(value_i2c);
  }
}

void setup()
{
  if (PRINT_SERIAL)
    Serial.println("Setup");

  Serial.begin(SERIAL_BUAD);
  Wire.begin(ADDRESS_I2C);
  Wire.onReceive(receiveEvent);
  motor.Enable();

  if (PRINT_SERIAL)
    Serial.println("Reload");

  while (!sw_f.get_stats())
  {
    motor.TurnLeft(PWM);
  }
  motor.Stop();
  motor.Disable();
  if (PRINT_SERIAL)
    Serial.println("End Setup");
}

void loop()
{
  if (value_i2c == KEY_FIRE)
  {
    if (PRINT_SERIAL)
      Serial.println("Fire");
    motor.Enable();
    rl.ON();
    while (!sw_b.get_stats())
    {
      motor.TurnRight(PWM);
    }
    rl.OFF();
    if (PRINT_SERIAL)
      Serial.println("Reload");
    while (!sw_f.get_stats())
    {
      motor.TurnLeft(PWM);
    }
    motor.Stop();
    value_i2c = NOT_WORKING;
    motor.Disable();
    if (PRINT_SERIAL)
      Serial.println("End");
  }
}