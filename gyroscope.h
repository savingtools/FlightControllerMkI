#include "Arduino.h"

#ifndef gyroscope_h
#define gyroscope_h

class gyroscope
{
    public:
        void initialize();
        void read();
        void lowPassFilter();

        uint16_t data[3];
        uint16_t prevData[3];
};

#endif