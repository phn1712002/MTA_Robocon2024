#include "Arduino.h"
#include "Wire.h"

void  write_i2c_value_float(const int &address, float const &value)
{
  Wire.beginTransmission(address);
  Wire.write((byte *)&value, sizeof(value));
  Wire.endTransmission();
}

void read_i2c_value_float(int const &howMany, float &result)
{
  const int from_pi = 5;
  const int from_adruino = 4;
  const int size_of_float = howMany == from_pi ? from_pi : from_adruino;
  byte bytes_array[4];

  if (size_of_float == from_pi)
  {
    for (int i = 0; i < from_pi; i++)
    {
      if (i == 0)
      {
        Wire.read(); // Delete one bytes from pi. Because pi send float with 5 bytes.
      }
      else
      {
        bytes_array[i - 1] = Wire.read();
      }
    }
    memcpy(&result, bytes_array, 4);
  }
  else if (size_of_float == from_adruino)
  {
    byte *p = (byte *)&result;
    for (unsigned int i = 0; i < from_adruino; i++)
      *p++ = Wire.read();
  }
}
