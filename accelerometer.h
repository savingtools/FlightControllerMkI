#include "Arduino.h"

#ifndef accelerometer_h
#define accelerometer_h

class accelerometer
{
    public:
        void initialize();
        void read();

        uint16_t data[3];
};

#endif