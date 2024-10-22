#include "Arduino.h"
#include "Wire.h"
#include "I2C_support_float.hpp"
#include "Line_ADC_AVR.hpp"
#include "Relay_support.hpp"
#include "Key_complete.hpp"

// PI3+ Connect Serial
#define BAUD_SERIAL 9600

// Error i2c
bool const i2c_debug_rest = true;
int const time_break_i2c = 25000U;

// Whistle
#define RL_IN 12
#define COUNT_WHISTLE_OC 3
#define COUNT_WHISTLE_WAIT 1
#define COUNT_WHISTLE_DEBUG 1
#define COUNT_WHISTLE_ERROR 3
#define DELAY_WHISTLE 100
Relay12V whistle(RL_IN);
void whistle_on_off(int count)
{
  for (int i = 0; i < count; i++)
  {
    whistle.ON();
    delay(DELAY_WHISTLE);
    whistle.OFF();
    delay(DELAY_WHISTLE);
  }
}

// Key complete
#define IN_KEY_COMPLETE A7
Key_Complete key(IN_KEY_COMPLETE);
float const time_delay_check_key = 50;

// Key Map
#define I2C_KEY_MAPS 30
#define SIZE_OF_MSG_KEY_MAPS 4
float const time_delay_reset_map = 1000;
int pass_wait = 0;
bool run_in_base_right = true;
bool read_status_run()
{

  whistle_on_off(COUNT_WHISTLE_WAIT);
  while (!key.check_complete())
    delay(time_delay_check_key);

  // RIGHT - LEFT?
  byte len = Wire.requestFrom(I2C_KEY_MAPS, SIZE_OF_MSG_KEY_MAPS);
  if (len == 0)
  {
    whistle_on_off(COUNT_WHISTLE_ERROR);
    return false;
  }

  else
  {
    float value = 0;
    read_i2c_value_float(SIZE_OF_MSG_KEY_MAPS, value);
    run_in_base_right = (bool)value;
  }

  // PASS WAIT
  len = Wire.requestFrom(I2C_KEY_MAPS, SIZE_OF_MSG_KEY_MAPS);
  if (len == 0)
  {
    whistle_on_off(COUNT_WHISTLE_ERROR);
    return false;
  }
  else
  {
    float value = 0;
    read_i2c_value_float(SIZE_OF_MSG_KEY_MAPS, value);
    pass_wait = (int)value;
  }
  whistle_on_off(COUNT_WHISTLE_WAIT);
  return true;
}

// Encoder Distance
#define ENA 2
#define ENB 3
#define COUNT_PER_ENCODER 20
#define DISTANCE_NO_TURN_DIR 0
float const wheel_diameter__mm = 100;
float const distance_wheel_per = wheel_diameter__mm * PI;
int ena = 0;
int enb = 0;
void count_ena()
{
  ena++;
}
void count_enb()
{
  enb++;
}
float distance_move(float en_l_begin, float en_r_begin)
{
  float distance_r = ((ena - en_r_begin) / (COUNT_PER_ENCODER)) * distance_wheel_per;
  float distance_l = ((enb - en_l_begin) / (COUNT_PER_ENCODER)) * distance_wheel_per;

  if (distance_l == 0)
    return distance_r;
  else if (distance_r == 0)
    return distance_l;

  return (distance_r + distance_l) / 2;
}

// PID
#define KP 50
#define KI 60
#define KD 12
float const vel_base_move = 291 * 0.6;
bool break_line = false;
long prevT = 0;
float eintegral = 0;
float eprev = 0;
float pid_compute(float e)
{
  // Delta T
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // dedt
  float dedt = (e - eprev) / (deltaT);
  eprev = e;

  // intergral e
  eintegral += e * deltaT;

  return KP * e + KD * dedt + KI * eintegral;
}
void reset_pid()
{
  prevT = micros();
  eintegral = 0;
  eprev = 0;
}


// Motor
#define I2C_MOTOR_LEFT 21
#define I2C_MOTOR_RIGHT 20
#define COUNT_STOP_MOTOR 10
float const rpm_max = 291;
float const rpm_min = 0;
float const motor_dir_rpm = 200;
float const motor_move_dir_rpm = 150;
float const motor_move_break = 150;
void motor_run(float vel_l, float vel_r)
{
  write_i2c_value_float(I2C_MOTOR_LEFT, vel_l);
  write_i2c_value_float(I2C_MOTOR_RIGHT, vel_r);
}
void motor_stop()
{
  for (int idx = 0; idx < COUNT_STOP_MOTOR; idx++)
  {
    motor_run(0, 0);
  }
}
float format_rpm(float rpm)
{
  rpm = rpm > rpm_min ? rpm : rpm_min;
  rpm = rpm < rpm_max ? rpm : rpm_max;
  return rpm;
}

// Line follow
int const reverse_dir = 1;
int const reverse_value = 0; // Line off led = 1, Line on line = 0
Detect_line d_line(13, 5, 6, 7, 8, 9, 10, 11, reverse_dir, reverse_value);

// Digital compass
#define I2C_COMPASS 8
#define SIZE_OF_MSG_COMPASS 4
float const time_start_digital = 3000;
float angle_error = 0;
bool read_angle_error()
{
  float angle_error_last = angle_error;
  byte len = Wire.requestFrom(I2C_COMPASS, SIZE_OF_MSG_COMPASS);
  if (len == 0)
    return false;
  else
  {
    read_i2c_value_float(SIZE_OF_MSG_COMPASS, angle_error);
    if (isnan(angle_error))
      angle_error = angle_error_last;
  }
  return true;
}

// FIRE
#define I2C_BASE_FIRE 11
float const time_delay_fire = 9000;
float const key_fire = 1;
int const size_buffer_fire = 1;
char const char_fire = '1';
char const char_off_fire = '0';
char buffer_fire;

// OC
#define I2C_OC 9
float const key_oc = 1;
float const delay_oc_and_whistle = 1000;

// Test
void measure_encoder()
{
  int en_r_begin = ena;
  int en_l_begin = enb;
  while (true)
  {
    float distance = distance_move(en_l_begin, en_r_begin);
    Serial.println(distance);
  }
}

// Function
void f_debug_audio_task()
{
  whistle_on_off(COUNT_WHISTLE_DEBUG);
}

void f_oc()
{
  write_i2c_value_float(I2C_OC, key_oc);
  delay(delay_oc_and_whistle);
  whistle_on_off(COUNT_WHISTLE_OC);
}

void f_wait()
{

  whistle_on_off(COUNT_WHISTLE_WAIT);
  while (!key.check_complete())
    delay(time_delay_check_key);
  whistle_on_off(COUNT_WHISTLE_WAIT);
}

void f_move_up(float distance_move_up = 0)
{
  distance_move_up = abs(distance_move_up);
  // Break line
  if (break_line)
  {
    while (!d_line.read_break_stop_line())
    {
      motor_run(motor_move_break, motor_move_break);
    }
    break_line = false;
  }
  // Follow line
  reset_pid();
  if (distance_move_up == 0)
  {
    while (!bool(abs(d_line.read_stop_line())))
    {
      float error = d_line.read_line();
      float u = pid_compute(error);
      float vel_run_l = format_rpm(vel_base_move + u);
      float vel_run_r = format_rpm(vel_base_move - u);
      motor_run(vel_run_l, vel_run_r);
    }
    break_line = true;
  }
  // Move distance
  else
  {
    int const en_r_begin = ena;
    int const en_l_begin = enb;
    float distance = 0;
    while (distance <= distance_move_up)
    {
      float error = d_line.read_line();
      float u = pid_compute(error);
      float vel_run_l = format_rpm(vel_base_move + u);
      float vel_run_r = format_rpm(vel_base_move - u);
      motor_run(vel_run_l, vel_run_r);
      distance = distance_move(en_l_begin, en_r_begin);
    }
  }
  motor_stop();
}

void f_dir_angle(float distance_turn_dir, int paramenter, bool inv = true)
{
  distance_turn_dir = abs(distance_turn_dir);
  if (distance_turn_dir > 0)
  {
    // TURN UP
    int const en_r_begin = ena;
    int const en_l_begin = enb;
    float distance = 0;
    while (distance < distance_turn_dir)
    {
      motor_run(motor_move_dir_rpm, motor_move_dir_rpm);
      distance = distance_move(en_l_begin, en_r_begin);
    }
    motor_stop();
  }

  // DIR
  bool right = paramenter < 0 ? true : false;
  read_angle_error();
  paramenter += angle_error;
  bool dir = true;
  while (dir)
  {
    read_angle_error();
    if (right)
    {
      if (angle_error <= paramenter)
        dir = false;
      if (inv)
        motor_run(motor_dir_rpm, 0);
      else
        motor_run(0, -motor_dir_rpm);
    }

    else
    {
      if (angle_error >= paramenter)
        dir = false;
      if (inv)
        motor_run(0, motor_dir_rpm);
      else
        motor_run(-motor_dir_rpm, 0);
    }
  }
  motor_stop();
}

void f_ai_fire(bool AI = false)
{

  // OUT BUTTON
  while (key.check_complete())
    delay(time_delay_check_key);
  if (AI)
  {
    bool fire = false;
    bool exit_fire = false;
    while (!exit_fire)
    {
      // Break AI Fire
      if (key.check_complete())
      {
        whistle_on_off(COUNT_WHISTLE_WAIT);
        fire = false;
        exit_fire = true;
        break;
      }

      // AI FIRE
      buffer_fire = char(Serial.read());
      if (buffer_fire == char_fire)
      {
        fire = true;
        break;
      }
    }

    if (fire)
    {
      buffer_fire = char_off_fire;
      // Fire
      write_i2c_value_float(I2C_BASE_FIRE, key_fire);
    }

    // Wait
    float time_current_fire = millis();
    while (millis() - time_current_fire < time_delay_fire)
    {
      if (key.check_complete())
      {
        whistle_on_off(COUNT_WHISTLE_WAIT);
        fire = false;
        exit_fire = true;
        break;
      }
    }
  }
  else
  {
    // FIRE
    write_i2c_value_float(I2C_BASE_FIRE, key_fire);

    // Wait
    float time_current_fire = millis();
    while (millis() - time_current_fire < time_delay_fire)
    {
      if (key.check_complete())
      {
        whistle_on_off(COUNT_WHISTLE_WAIT);
        break;
      }
    }
  }
}

// SETUP
void setup()
{
  // Config Serial
  Serial.begin(BAUD_SERIAL);
  // End Serial

  // I2C
  Wire.begin();
  Wire.setWireTimeout(time_break_i2c, i2c_debug_rest);
  // End I2C

  // Encoder distance
  pinMode(ENA, INPUT);
  attachInterrupt(0, count_ena, RISING);

  pinMode(ENB, INPUT);
  attachInterrupt(1, count_enb, RISING);
  // End Encoder

  // Wait Digital
  delay(time_start_digital);
}

// TASK
void base_right(int pass_wait = 0)
{
  if (pass_wait == 0)
  {
    // C1
    f_move_up(2000);
    f_move_up();
    f_move_up();
    f_move_up();
    f_oc();
    f_move_up();
    f_move_up();
    // END

    // D1
    f_wait();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_oc();
    f_move_up();
    f_dir_angle(900, 85, true);
    f_move_up();
    // END

    // A1
    f_wait();
    f_move_up();
    f_oc();
    f_move_up(12000);
    f_dir_angle(0, -85, true);
    f_dir_angle(0, -85, false);
    f_move_up();
    f_move_up();
    // END

    // FIRE
    f_wait();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_dir_angle(2500, -60, false);
    f_ai_fire(false);
    
    // END
  }
  
  if (pass_wait == 1)
  {
    // C1
    f_move_up(2000);
    f_move_up();
    f_move_up();
    f_move_up();
    f_move_up();
    f_move_up();
    // END

    // D1
    f_wait();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_oc();
    f_move_up();
    f_dir_angle(900, 85, true);
    f_move_up();
    // END

    // A1
    f_wait();
    f_move_up();
    f_oc();
    f_move_up(12000);
    f_dir_angle(0, -85, true);
    f_dir_angle(0, -85, false);
    f_move_up();
    f_move_up();
    // END

    // FIRE
    f_wait();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_dir_angle(2500, -60, false);
    f_ai_fire(false);
    
    // END
  }

  if (pass_wait == 2)
  {
    // A1
    f_move_up(2000);
    f_move_up();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_wait();
    f_dir_angle(900, -85, true);
    f_move_up();
    f_oc();
    f_move_up();
    // END

    // FIRE
    f_wait();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_dir_angle(2500, -60, false);
    f_ai_fire(false);
    
    // END
  }

  if (pass_wait == 3)
  {
    // A1
    f_move_up(2000);
    f_move_up();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_dir_angle(900, -85, true);
    f_move_up();
    f_move_up();
    // END

    // FIRE
    f_dir_angle(900, 85, true);
    f_move_up();
    f_dir_angle(2500, -60, false);
    f_ai_fire(false);
    
    // END
  }
}

void base_left(int pass_wait = 0)
{
  if (pass_wait == 0)
  {
    // C1
    f_move_up(2000);
    f_move_up();
    f_move_up();
    f_move_up();
    f_oc();
    f_move_up();
    f_move_up();
    // END

    // D1
    f_wait();
    f_dir_angle(900, -85, true);
    f_move_up();
    f_oc();
    f_move_up();
    f_dir_angle(900, -85, true);
    f_move_up();
    // END

    // A1
    f_wait();
    f_move_up();
    f_oc();
    f_move_up(12000);
    f_dir_angle(0, -85, true);
    f_dir_angle(0, -85, false);
    f_move_up(); 
    f_move_up();
    // END

    // FIRE
    f_wait();
    f_dir_angle(700, -85, true);
    f_move_up();
    f_dir_angle(2500, 50, false);
    f_dir_angle(0, 10, true);
    f_ai_fire(false);
    
    // END
  }
  
  if (pass_wait == 1)
  {
    // C1
    f_move_up(2000);
    f_move_up();
    f_move_up();
    f_move_up();
    f_move_up();
    f_move_up();
    // END

    // D1
    f_wait();
    f_dir_angle(900, -85, true);
    f_move_up();
    f_oc();
    f_move_up();
    f_dir_angle(900, -85, true);
    f_move_up();
    // END

    // A1
    f_wait();
    f_move_up();
    f_oc();
    f_move_up(12000);
    f_dir_angle(0, -85, true);
    f_dir_angle(0, -85, false);
    f_move_up();
    f_move_up();
    // END

    // FIRE
    f_wait();
    f_dir_angle(700, -85, true);
    f_move_up();
    f_dir_angle(2500, 50, false);
    f_dir_angle(0, 10, true);
    f_ai_fire(false);
    
    // END
  }

  if (pass_wait == 2)
  {

    // A1
    f_move_up(2000);
    f_move_up();
    f_dir_angle(900, -85, true);
    f_move_up();
    f_wait();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_oc();
    f_move_up();
    // END

    // FIRE
    f_wait();
    f_dir_angle(700, -85, true);
    f_move_up();
    f_dir_angle(2500, 50, false);
    f_dir_angle(0, 10, true);
    f_ai_fire(false);
    
    // END
  }

  if (pass_wait == 3)
  {

    // A1
    f_move_up(2000);
    f_move_up();
    f_dir_angle(900, -85, true);
    f_move_up();
    f_dir_angle(900, 85, true);
    f_move_up();
    f_move_up();
    // END

    // FIRE
    f_dir_angle(700, -85, true);
    f_move_up();
    f_dir_angle(2500, 50, false);
    f_dir_angle(0, 10, true);
    f_ai_fire(false);
    
    // END
  }

}

bool end_task = false;
// LOOP
void loop()
{
  if (!end_task)
  {
    bool check = read_status_run();
    if (check)
    {
      if (run_in_base_right)
      {
        base_right(pass_wait);
      }

      else
      {
        base_left(pass_wait);
      }

      end_task = true;
    }
    else
    {
      delay(time_delay_reset_map);
      end_task = false;
    }
  }
}