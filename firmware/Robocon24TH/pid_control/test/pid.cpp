#include "Arduino.h"
#include "BTS7960.h"
#include "Wire.h"
#include "I2C_support_float.hpp"

// Encoder
#define ENCA 3
#define ENCB 2

// Motor
#define EN 8
#define R_PWM 10
#define L_PWM 9
#define MAX_PWM 255
#define COUNTS_PER_REV_ENCODER 16
#define GEAR_BOX 13.6875
BTS7960 motorController(EN, L_PWM, R_PWM);

// I2C Serial
#define ADDRESS_I2C 20
#define SERIAL_BUAD 9600
#define PRINT_SEARIAL true

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float rpm_current_m2filt = 0;
float rpm_current_m2prev = 0;

float eintegral = 0;
float e_last = 0;

// PID and Target
#define KP 10
#define KI 10
#define KD 0.01
float rpm_request = 0;

void readEncoder()
{
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0)
  {
    // If B is high, increment forward
    increment = 1;
  }
  else
  {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}

void receiveEvent(int howMany)
{
  read_i2c_value_float(howMany, rpm_request);
}

void setup()
{
  Serial.begin(SERIAL_BUAD);
  Wire.begin(ADDRESS_I2C);
  Wire.onReceive(receiveEvent);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  motorController.Enable();

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder, RISING);
}

void loop()
{
  // Read Encoder Vel
  int pos = 0;
  float velocity2 = 0;
  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts();

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  posPrev = pos;
  prevT = currT;

  float rpm2 = float(velocity2 / float(COUNTS_PER_REV_ENCODER * GEAR_BOX) * 60.0);

  rpm_current_m2filt = 0.854 * rpm_current_m2filt + 0.0728 * rpm2 + 0.0728 * rpm_current_m2prev;
  rpm_current_m2prev = rpm2;

  // End read Encoder Vel

  // rpm_request = 100*(sin(currT/1e6)>0);
  float e = rpm_request - rpm_current_m2filt;
  eintegral = eintegral + e * deltaT;
  float de = (e_last - e) / deltaT;
  float u = KP * e + KI * eintegral + KD * de;
  e_last = e;

  bool turn_right = u < 0 ? false : true;
  int pwr = (int)fabs(u);

  if (pwr > 255)
  {
    pwr = 255;
  }

  if (pwr == 0)
    motorController.Stop();
  else
  {
    if (turn_right)
      motorController.TurnRight(pwr);
    else
      motorController.TurnLeft(pwr);
  }

  if (PRINT_SEARIAL)
  {
    Serial.print(rpm_request);
    Serial.print(" ");
    Serial.print(rpm_current_m2filt);
    Serial.println();
  }
}