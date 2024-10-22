#include "Arduino.h"

class Detect_line
{
public:
    Detect_line(int pin0, int pin1, int pin2, int pin3, int pin4, int pin5, int pin6, int pin7, int reverse_dir, int reverse_value);
    int *getAllStatus();
    float read_line();
    void print_value();
    int read_stop_line();
    int *getStatusStopLine();
    int *getStatusStopLineDir();
    bool read_stop_line_dir();
    bool read_break_stop_line();

private:
    int read_value(int pin, int reverse_value);
    int __pin0;
    int __pin1;
    int __pin2;
    int __pin3;
    int __pin4;
    int __pin5;
    int __pin6;
    int __pin7;
    int __reverse_dir;
    int __reverse_value;
    int const size_eyes_line = 8;
    int *__status = new int[size_eyes_line];
};

Detect_line::Detect_line(int pin0, int pin1, int pin2, int pin3, int pin4, int pin5, int pin6, int pin7, int reverse_dir, int reverse_value)
{
    pinMode(pin0, INPUT);
    pinMode(pin1, INPUT);
    pinMode(pin2, INPUT);
    pinMode(pin3, INPUT);
    pinMode(pin4, INPUT);
    pinMode(pin5, INPUT);
    pinMode(pin6, INPUT);
    pinMode(pin7, INPUT);

    this->__pin0 = pin0;
    this->__pin1 = pin1;
    this->__pin2 = pin2;
    this->__pin3 = pin3;
    this->__pin4 = pin4;
    this->__pin5 = pin5;
    this->__pin6 = pin6;
    this->__pin7 = pin7;

    this->__reverse_dir = reverse_dir;
    this->__reverse_value = reverse_value;
}

int *Detect_line::getStatusStopLine()
{
    int *status = new int[2];
    this->__status[0] = read_value(this->__pin0, this->__reverse_value);
    this->__status[7] = read_value(this->__pin7, this->__reverse_value);

    status[0] = this->__status[0];
    status[1] = this->__status[7];

    return status;
}

int *Detect_line::getStatusStopLineDir()
{
    int *status = new int[2];
    this->__status[3] = read_value(this->__pin3, this->__reverse_value);
    this->__status[4] = read_value(this->__pin4, this->__reverse_value);

    status[0] = this->__status[3];
    status[1] = this->__status[4];

    return status;
}

bool Detect_line::read_stop_line_dir()
{
    this->getStatusStopLineDir();
    int check = this->__reverse_value == 0 ? 1 : 0;
    if (this->__status[3] == check)
        return true;
    else if (this->__status[4] == check)
        return true;
    return false;
}

int *Detect_line::getAllStatus()
{
    this->__status[0] = read_value(this->__pin0, this->__reverse_value);
    this->__status[1] = read_value(this->__pin1, this->__reverse_value);
    this->__status[2] = read_value(this->__pin2, this->__reverse_value);
    this->__status[3] = read_value(this->__pin3, this->__reverse_value);
    this->__status[4] = read_value(this->__pin4, this->__reverse_value);
    this->__status[5] = read_value(this->__pin5, this->__reverse_value);
    this->__status[6] = read_value(this->__pin6, this->__reverse_value);
    this->__status[7] = read_value(this->__pin7, this->__reverse_value);
    return this->__status;
}

float Detect_line::read_line()
{
    this->getAllStatus();
    int pairs[][2] = {{0, 1}, {6, 7}, {1, 2}, {5, 6}, {3, 4}, {2, 3}, {4, 5}};
    float size_zone_value[7] = {-1.5, 1.5, -1, 1, 0, -0.5, 0.5};
    int numPairs = sizeof(pairs) / sizeof(pairs[0]);
    for (int i = 0; i < numPairs; i++)
    {
        int firstIndex = pairs[i][0];
        int secondIndex = pairs[i][1];
        int value_sensor = (this->__status[firstIndex] + this->__status[secondIndex]) >= 1 ? 1 : 0;
        if (value_sensor != 0)
            return this->__reverse_dir * size_zone_value[i];
    }
    return 0;
}

int Detect_line::read_value(int pin, int reverse_value)
{
    if (reverse_value)
        return digitalRead(pin) == 0 ? 0 : 1;
    else
        return digitalRead(pin) == 0 ? 1 : 0;
}

int Detect_line::read_stop_line()
{
    this->getStatusStopLine();
    int check = this->__reverse_value == 0 ? 1 : 0;
    if (this->__status[0] == check)
        return -1 * this->__reverse_dir;
    else if (this->__status[7] == check)
        return this->__reverse_dir;
    else
        return 0;
}

bool Detect_line::read_break_stop_line()
{
    this->getStatusStopLine();
    int sum = 0;
    int check = this->__reverse_value == 0 ? 1 : 0;
    if (this->__status[0] == check) sum++;
    if (this->__status[7] == check) sum++;
    return sum >= 1 ? false: true;
}

void Detect_line::print_value()
{
    this->getAllStatus();
    Serial.print(this->__status[7]);
    Serial.print(this->__status[6]);
    Serial.print(this->__status[5]);
    Serial.print(this->__status[4]);
    Serial.print(this->__status[3]);
    Serial.print(this->__status[2]);
    Serial.print(this->__status[1]);
    Serial.print(this->__status[0]);
    Serial.println("");
}