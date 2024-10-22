#include "Arduino.h"

class Relay12V
{
  public:
    Relay12V(uint8_t pin);
    void ON();
    void OFF();
  private:
    uint8_t __pin;
};

Relay12V::Relay12V(uint8_t pin)
{
  pinMode(pin, OUTPUT);
  this->__pin = pin;  
}

void Relay12V::ON()
{
  digitalWrite(this->__pin, HIGH);
}

void Relay12V::OFF()
{
  digitalWrite(this->__pin, LOW);
}

