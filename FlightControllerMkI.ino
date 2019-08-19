#include "quadcopter.h"


quadcopter myQuad;

void setup()
{
    myQuad.initialize();
}

void loop()
{
    switch (myQuad.state)
    {
    case POWERED:
        myQuad.escOn();
        myQuad.stabilize();
        break;
    default:
        break;
    }
}

/*
ISR(PCINT0_vect)
{
    myQuad.isrCtrlSignal();
}
*/