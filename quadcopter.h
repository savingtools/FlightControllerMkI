#include "arduino.h"
#include "accelerometer.h"
#include "gyroscope.h"

#ifndef quadcopter_h
#define quadcopter_h

// Directions
#define X 0
#define Y 1
#define Z 2

// Communication Channels
#define COMM_CH1                0
#define COMM_CH2                1
#define COMM_CH3                2
#define COMM_CH4                3
#define COMM_N                  4

// Control Inputs - X & Y Velocity, Z Acceleration, Z Angular Velocity
#define V_X                     0   // COMM_CH2
#define V_Y                     1   // COMM_CH4
#define A_Z                     2   // COMM_CH3
#define W_Z                     3   // COMM_CH1

// ESC signals driven by Arduino pins 4-7 -> ATmega328p pins PD4-7
#define MASK_ESC_1              (1 << PORT4)
#define MASK_ESC_2              (1 << PORT5)
#define MASK_ESC_3              (1 << PORT6)
#define MASK_ESC_4              (1 << PORT7)
#define MASK_ESC                (MASK_ESC_1 | MASK_ESC_2 | MASK_ESC_3 | MASK_ESC_4)

// Controller inputs on Arduino pins 8-11 -> ATmega328p pins PB0-3
#define MASK_CTRL_1             (1 << PORT0)
#define MASK_CTRL_2             (1 << PORT1)
#define MASK_CTRL_3             (1 << PORT2)
#define MASK_CTRL_4             (1 << PORT3)
#define MASK_CTRL               (MASK_CTRL_1 | MASK_CTRL_2 | MASK_CTRL_3 | MASK_CTRL_4)

#define CYCLES_PER_SECOND       250
#define CTRL_RESOLUTION         500 // Number of discrete input values between 0 and maximum
#define SPEED_RESOLUTION        10 // Number of allowed discrete velocities between 0 and maximum
#define ROTATION_SPEED_XY       10 // DPS - degrees per second
#define ROTATION_SPEED_Z        360
#define CYCLES_PER_DEGREE       (CYCLES_PER_SECOND / ROTATION_SPEED_XY)

enum states
{
    STANDBY,
    READY,
    POWERED
};

class quadcopter
{
    public:
        uint8_t state;

        void initialize();
        void stabilize();
        void escOn();
        void esc1Off();
        void esc2Off();
        void esc3Off();
        void esc4Off();
        void isrCtrlSignal();

    private:
        uint8_t i;

        uint64_t currentTime;

        uint16_t ctrlSignal[COMM_N];
        uint64_t ctrlSignalStart[COMM_N];
        uint8_t ctrlSignalState[COMM_N];
        uint16_t ctrlInput[COMM_N];

        uint16_t rotationTarget[3];
        uint16_t rotationCounter[3];
        uint16_t rotationSetpoint[3];
        float rotationSpeed[3];
        float acceleration[3];
        float thrustEstimate[3];
        float rotationSpeedError[3];

        gyroscope quadGyro;
        accelerometer quadAccel;
};

#endif