#include "Arduino.h"
#include "Keypad.h"
#include "Wire.h"
#include "I2C_support_float.hpp"
#include "LiquidCrystal_I2C.h"

#define ADDRESS_I2C 30

#define BAUD_SERIAL 9600
#define PRINT_SERIAL true

// LCD 
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// STATUS
bool run_in_right = true;
int pass_wait = 0;
String str = "R-0";


// Error i2c
bool const i2c_debug_rest = true;
int const time_break_i2c = 25000U;
int count_send_status = 0;
void requestEvent()
{
  if(count_send_status == 0)
  {
    float value = (int)run_in_right;
    Wire.write((byte *)&value, sizeof(value));
    count_send_status++;
  }
  else
  {
    float value = (int)pass_wait;
    Wire.write((byte *)&value, sizeof(value));
    count_send_status--;
  }
}

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'A','B','C','D'},
  {'a','b','c','d'},
};
byte rowPins[ROWS] = {7, 6}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

void setup(){
  Serial.begin(BAUD_SERIAL);
  Serial.begin(BAUD_SERIAL);
  Wire.begin(ADDRESS_I2C);
  Wire.onRequest(requestEvent);
  Wire.setWireTimeout(time_break_i2c, i2c_debug_rest);

  // LCD
  lcd.init();                 
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("DIR - PASS");
  lcd.setCursor(0,1);
  lcd.print(str);
}
  
void loop(){
  char customKey = customKeypad.getKey();
  
  if (customKey == 'a')
  {
    run_in_right = true;
    pass_wait = 0;
    str = "R-0";
    if(PRINT_SERIAL) Serial.println(str);
    
    lcd.setCursor(0,1);
    lcd.print(str);
  }
  if (customKey == 'b')
  {
    run_in_right = true;
    pass_wait = 1;
    str = "R-1";
    if(PRINT_SERIAL) Serial.println(str);

    lcd.setCursor(0,1);
    lcd.print(str);
  }
  if (customKey == 'c')
  {
    run_in_right = true;
    pass_wait = 2;
    str = "R-2";
    if(PRINT_SERIAL) Serial.println(str);

    lcd.setCursor(0,1);
    lcd.print(str);
  }
  if (customKey == 'd')
  {
    run_in_right = true;
    pass_wait = 3;
    str = "R-3";
    if(PRINT_SERIAL) Serial.println(str);

    lcd.setCursor(0,1);
    lcd.print(str);
  }

  if (customKey == 'A')
  {
    run_in_right = false;
    pass_wait = 0;
    str = "L-0";
    if(PRINT_SERIAL) Serial.println(str);

    lcd.setCursor(0,1);
    lcd.print(str);
  }
  if (customKey == 'B')
  {
    run_in_right = false;
    pass_wait = 1;
    str = "L-1";
    if(PRINT_SERIAL) Serial.println(str);

    lcd.setCursor(0,1);
    lcd.print(str);
  }
  if (customKey == 'C')
  {
    run_in_right = false;
    pass_wait = 2;
    str = "L-2";
    if(PRINT_SERIAL) Serial.println(str);

    lcd.setCursor(0,1);
    lcd.print(str);
  }
  if (customKey == 'D')
  {
    run_in_right = false;
    pass_wait = 3;
    str = "L-3";
    if(PRINT_SERIAL) Serial.println(str);

    lcd.setCursor(0,1);
    lcd.print(str);
  }
   
}