#include "Arduino.h"
#include "Servo.h"
#include "Wire.h"
#include "I2C_support_float.hpp"

#define PRINT_SERIAL true
#define SERIAL_BUAD 9600

// MOTOR
#define IN_MOTOR 12
float const delay_run = 15;
float const count = 1;
Servo servo;

// I2C
#define ADDRESS_I2C 9
#define KEY_COMPLETE 1
#define NOT_WORKING -1
float i2c_value;
void receiveEvent(int howMany)
{
  read_i2c_value_float(howMany, i2c_value);
  if (PRINT_SERIAL)
    Serial.println(i2c_value);
}

void setup()
{

  Serial.begin(SERIAL_BUAD);
  servo.attach(IN_MOTOR);
  Wire.begin(ADDRESS_I2C);
  Wire.onReceive(receiveEvent);

  // ON
  for (int angle = 0; angle < 45; angle++)
  {
    servo.write(angle);
    delay(delay_run);
  }
}

void loop()
{
  if (i2c_value == KEY_COMPLETE)
  {
    i2c_value = NOT_WORKING;
    for (int i = 0; i < count; i++)
    {
      for (int angle = 45; angle < 90; angle++)
      {
        servo.write(angle);
        delay(delay_run);
      }
      for (int angle = 90; angle > 0; angle--)
      {
        servo.write(angle);
        delay(delay_run);
      }
      for (int angle = 0; angle < 45; angle++)
      {
        servo.write(angle);
        delay(delay_run);
      }
    }
  }
}
