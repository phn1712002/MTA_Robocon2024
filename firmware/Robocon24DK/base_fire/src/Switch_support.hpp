#include "Arduino.h"

class Switch
{
  private:
  uint8_t __pin;

  public:
    Switch(uint8_t pin);
    bool get_stats();
};
  
Switch::Switch(uint8_t pin)
{
  pinMode(pin, INPUT_PULLUP);
  this->__pin = pin;
}

bool Switch::get_stats()
{
  return digitalRead(this->__pin) == 1 ? false : true;
}
