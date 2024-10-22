#include "Arduino.h"

float ConvertJoystickToNeg1to1(const float &value, const uint16_t &joystickMax = 255)
{
    float res = float(value / joystickMax); // Convert to -0.5, 0.5 range (nearly, because of the trim)
    
    return 2 * res - 1;
}
