
#include "Arduino.h"
#include "PS2X_lib.h"
#include "I2C_support_float.hpp"
#include "Kinematics_three_omni.hpp"
#include "Joystick.hpp"
#include "Wire.h"
#include "SoftwareSerial.h"
#include "math.h"

// Serial
#define BAUD_SERIAL 9600
#define PRINT_SERIAL true

// RF LoRa
#define BAUD_LORA 9600
#define TX 5
#define RX 4
char const char_lora_run = '1';
SoftwareSerial Lora(RX, TX);

// GamePad
#define PS2_DAT 13
#define PS2_CMD 11
#define PS2_SEL 10
#define PS2_CLK 12
PS2X ps2x;

// Omni
#define I2C_MOTOR_1 20
#define I2C_MOTOR_2 21
#define I2C_MOTOR_3 22
float dx, dy, dRot;
float const max_motor_rpm = 291;
float const slowdRot_Fire = 0.15f;
float const slowdRot = 0.5f;
float const slowdX = 0.3f;
float const slowdY = 0.2f;
float const slowdRot_digital_compass = 0.2f;
float const angle_error_size_zone = 10;
int sign_rotation_digital_compass = 0;
Kinematics::rpm all_rpm_current;

// Physical dimensions of the platform
const auto platform_wheel_offset_meters = 0.104f;
const auto wheel_radius_meters = 100.0f / 1000 / 2;
const auto max_Wheel_Rotation_Velocity_RadiansPerSec = max_motor_rpm * (2 * M_PI) / 60;
const auto max_Translation_Velocity_MetersPerSec = wheel_radius_meters * max_Wheel_Rotation_Velocity_RadiansPerSec;
const auto max_Platform_Rotation_Velocity_RadiansPerSec = max_Translation_Velocity_MetersPerSec / platform_wheel_offset_meters;
const auto max_Platform_Rotation_Velocity_DegreesPerSec = max_Platform_Rotation_Velocity_RadiansPerSec * 180 / M_PI;
Kinematics kinematics(platform_wheel_offset_meters, wheel_radius_meters);

// Electric Cylinder
#define I2C_CYLINDER 10
bool control_cylinder;
float value_control_cylinder_current = 0;
float value_control_cylinder_last = 0;
float const value_up_cylinder = -1;
float const value_down_cylinder = 1;
float const value_stop_cylinder = 0;

// Base Fire
#define I2C_FIRE 11
float const value_fire = 1;
float const time_stop_fire = 7000;
float time_current_fire = 0;

// Digital compass
#define I2C_COMPASS 8
#define SIZE_OF_MSG 4
float angle_error = 0;
float angle_error_last = 0;
float angle_fix = 0;
bool read_angle_error()
{
  angle_error_last = angle_error;
  byte len = Wire.requestFrom(I2C_COMPASS, SIZE_OF_MSG);
  if (len == 0)
  {
    return false;
  }
  else
  {
    read_i2c_value_float(SIZE_OF_MSG, angle_error);
    if (isnan(angle_error))
      angle_error = angle_error_last;
  }
  return true;
}

// Error i2c
bool const i2c_debug_rest = true;
int const time_break_i2c = 25000U;

void setup()
{
  // I2C
  Wire.begin();
  Wire.setWireTimeout(time_break_i2c, i2c_debug_rest);
  // End I2C

  // Config Serial
  Serial.begin(BAUD_SERIAL);
  // End Serial

  // Config Lora
  Lora.begin(BAUD_LORA);
    
  // End Lora

  // Config Gamepad
  if (PRINT_SERIAL)
    Serial.println("Config gamepad");
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  ps2x.read_gamepad(true, 1);
  delay(1000);
  ps2x.read_gamepad(false, 0);
  if (PRINT_SERIAL)
    Serial.println("End config gamepad");
  // End Gamepad
}

void loop()
{
  // PS2 read
  ps2x.read_gamepad();

  if (ps2x.NewButtonState())
  {
    // RF 
    if (ps2x.Button(PSB_START))
    {
        Lora.write(char_lora_run);
        if(PRINT_SERIAL) Serial.println("LORA SEND");
    }

    // Fire
    if (ps2x.Button(PSB_R1) || ps2x.Button(PSB_L1))
    {
      if (time_current_fire == 0)
      {
        time_current_fire = millis();
        write_i2c_value_float(I2C_FIRE, value_fire);
      }
      else if (millis() - time_current_fire > time_stop_fire)
      {
        time_current_fire = millis();
        Wire.clearWriteError();
        write_i2c_value_float(I2C_FIRE, value_fire);
      }
      if (PRINT_SERIAL)
        Serial.println("Fire_R1_L1");
    }
    // End fire
  }

  // Fix angle
  if (ps2x.Button(PSB_L2))
  {
    if (read_angle_error())
      angle_fix = angle_error;
    if (PRINT_SERIAL)
      Serial.println("L2_FIX_ANGLE");
  }
  // End fix angle

  // Cylinder
  control_cylinder = false;
  if (ps2x.Button(PSB_TRIANGLE)) // UP
  {
    if (PRINT_SERIAL)
      Serial.println("Cylinder_UP");
    control_cylinder = true;
    value_control_cylinder_current = value_up_cylinder;
    if (value_control_cylinder_last != value_control_cylinder_current)
      write_i2c_value_float(I2C_CYLINDER, value_up_cylinder);
  }
  if (ps2x.Button(PSB_CROSS)) // DOWN
  {
    if (PRINT_SERIAL)
      Serial.println("Cylinder_DOWN");
    control_cylinder = true;
    value_control_cylinder_current = value_down_cylinder;
    if (value_control_cylinder_last != value_control_cylinder_current)
      write_i2c_value_float(I2C_CYLINDER, value_down_cylinder);
  }
  if (!control_cylinder) // STOP
  {
    value_control_cylinder_current = value_stop_cylinder;
    write_i2c_value_float(I2C_CYLINDER, value_stop_cylinder);
  }
  value_control_cylinder_last = value_control_cylinder_current;
  // End cylinder

  dx = ConvertJoystickToNeg1to1(ps2x.Analog(PSS_LX)) * max_Translation_Velocity_MetersPerSec;
  dy = ConvertJoystickToNeg1to1(ps2x.Analog(PSS_LY)) * max_Translation_Velocity_MetersPerSec;
  dRot = -ConvertJoystickToNeg1to1(ps2x.Analog(PSS_RX)) * max_Platform_Rotation_Velocity_DegreesPerSec * slowdRot;

  if (ps2x.Button(PSB_SQUARE))
  {
    if (PRINT_SERIAL)
      Serial.println("SQUARE_DIR_LEFT");
    dRot = max_Platform_Rotation_Velocity_DegreesPerSec * slowdRot_Fire;
  }
  if (ps2x.Button(PSB_CIRCLE))
  {
    if (PRINT_SERIAL)
      Serial.println("CICRCLE_DIR_RIGHT");
    dRot = -max_Platform_Rotation_Velocity_DegreesPerSec * slowdRot_Fire;
  }

  if (ps2x.Button(PSB_PAD_LEFT))
  {
    if (PRINT_SERIAL)
      Serial.println("LEFT");
    dx = -max_Translation_Velocity_MetersPerSec * slowdX;
    dy = 0;
  }
  if (ps2x.Button(PSB_PAD_RIGHT))
  {
    if (PRINT_SERIAL)
      Serial.println("RIGHT");
    dx = max_Translation_Velocity_MetersPerSec * slowdX;
    dy = 0;
  }

  if (ps2x.Button(PSB_PAD_UP))
  {
    if (PRINT_SERIAL)
      Serial.println("UP");
    dy = -max_Translation_Velocity_MetersPerSec * slowdY;
    dx = 0;
  }
  if (ps2x.Button(PSB_PAD_DOWN))
  {
    if (PRINT_SERIAL)
      Serial.println("DOWN");
    dy = max_Translation_Velocity_MetersPerSec * slowdY;
    dx = 0;
  }

  if (ps2x.Button(PSB_R2))
  {
    if (read_angle_error())
    {
      if (PRINT_SERIAL)
        Serial.println("R2_COMPASS");
      int angle_error_correction = angle_error - angle_fix;
      sign_rotation_digital_compass = angle_error_correction > 0 ? -1 : 1;
      if (abs(angle_error_correction) > 180)
        sign_rotation_digital_compass = -sign_rotation_digital_compass;
      dRot = sign_rotation_digital_compass * min(abs(float(angle_error_correction / angle_error_size_zone)), 1) * max_Platform_Rotation_Velocity_DegreesPerSec * slowdRot_digital_compass;
    }
  }


  all_rpm_current = kinematics.getRPM(dx, dy, dRot);

  write_i2c_value_float(I2C_MOTOR_1, all_rpm_current.motorRpm[0]);
  write_i2c_value_float(I2C_MOTOR_2, all_rpm_current.motorRpm[1]);
  write_i2c_value_float(I2C_MOTOR_3, all_rpm_current.motorRpm[2]);

  if (PRINT_SERIAL)
      Serial.println("RUN");
  // End three omni
  delay(5);
}
