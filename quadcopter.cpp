#include "quadcopter.h"

void quadcopter::initialize()
{
    // Set ESC signal pins as outputs
    DDRD |= MASK_ESC;

    // Set controller signal pins as inputs
    DDRB |= MASK_CTRL;

    // Enable pin change interrupts on the controller signal pins
    PCICR |= PCIE0;
    PCMSK0 |= (1 << PCINT0);
    PCMSK0 |= (1 << PCINT1);
    PCMSK0 |= (1 << PCINT2);
    PCMSK0 |= (1 << PCINT3);

    quadGyro.initialize();
    quadAccel.initialize();
}

void quadcopter::stabilize()
{
    // Decide on setpoint

    /*
    Speed resolution +/-10
    To go from speed 0 to 1:
    Set pitch setpoint to 10dps for 0.1s
    Speed 10 means 10deg rotation, rotate for 1sec at 10dps
    */

    // Scale control inputs to speed control resolution
    ctrlInput[V_X] = (ctrlSignal[COMM_CH2] - 1500) * SPEED_RESOLUTION / CTRL_RESOLUTION;
    ctrlInput[V_Y] = (ctrlSignal[COMM_CH4] - 1500) * SPEED_RESOLUTION / CTRL_RESOLUTION;
    ctrlInput[A_Z] = (ctrlSignal[COMM_CH3] - 1000) * SPEED_RESOLUTION / CTRL_RESOLUTION;
    ctrlInput[W_Z] = (ctrlSignal[COMM_CH1] - 1500) * SPEED_RESOLUTION / CTRL_RESOLUTION;

    // Set Y and X rotation setpoint to approach desired X and Y velocity
    for (i = 0; i < Z; i++)
    {
        rotationTarget[Y-i] = CYCLES_PER_DEGREE * ctrlInput[V_X+i];
        if (rotationCounter[Y-i] < rotationTarget[Y-i])
        {
            rotationSetpoint[Y-i] = ROTATION_SPEED_XY;
            rotationCounter[Y-i]++;
        }
        else if (rotationCounter[Y-i] > rotationTarget[Y-i])
        {
            rotationSetpoint[Y-i] = -ROTATION_SPEED_XY;
            rotationCounter[Y-i]--;
        }
        else
        {
            rotationSetpoint[Y-i] = 0;
        }
    }

    // Set A_Z

    rotationSetpoint[Z] = ctrlInput[W_Z] * (ROTATION_SPEED_Z / SPEED_RESOLUTION);


    // Read Gyroscope
    quadGyro.read();
    quadGyro.lowPassFilter();
    for (i = 0; i < 3; i++) rotationSpeed[i] = quadGyro.data[i];

    // Read Accelerometer
    quadAccel.read();
    // Filter?
    for (i = 0; i < 3; i++) acceleration[i] = quadAccel.data[i] - thrustEstimate[i];


    // Compute deviation from setpoint
    for (i = 0; i < 3; i++) rotationSpeedError[i] = rotationSpeed[i] - rotationSetpoint[i];


    // Compute pid corrections
    // Update ESC inputs
}

void quadcopter::isrCtrlSignal()
{
    currentTime = micros();

    for (i = COMM_CH1; i < COMM_N; i++)
    {
        // Check ports 0-3
        if((PINB & (MASK_CTRL_1 << i)) && !ctrlSignalState[i])
        {
            ctrlSignalState[i] = 1;
            ctrlSignalStart[i] = currentTime;
        }
        else if (ctrlSignalState[i])
        {
            ctrlSignal[i] = currentTime - ctrlSignalStart[i];
            ctrlSignalState[i] = 0;
        }
    }
}

void quadcopter::escOn()
{
    PORTD |= MASK_ESC;
}

void quadcopter::esc1Off()
{
    PORTD &= ~MASK_ESC_1;
}

void quadcopter::esc2Off()
{
    PORTD &= ~MASK_ESC_2;
}

void quadcopter::esc3Off()
{
    PORTD &= ~MASK_ESC_3;
}

void quadcopter::esc4Off()
{
    PORTD &= ~MASK_ESC_4;
}