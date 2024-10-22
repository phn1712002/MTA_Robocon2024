#include "Arduino.h"

class Key_Complete
{
  private:
    int __pin;

  public:
    Key_Complete(int);  
    bool check_complete(float, bool);
};

Key_Complete::Key_Complete(int pin)
{
  pinMode(pin, INPUT_PULLUP);
  this->__pin = pin;
}

bool Key_Complete::check_complete(float threshold_v = 500, bool reverse = false)
{
  if(reverse)
    return analogRead(this->__pin) > threshold_v ? false : true;
  else return analogRead(this->__pin) > threshold_v ? true : false;
}

  