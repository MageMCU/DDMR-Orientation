//
// Carpenter Software
// File: Class L298N.h (Archived Version)
// Folder: DDMR-Orientation/DDMR Code/include/
//
// Purpose: Github Depository (MageMCU)
//
// Algebra OOP Library
// The math is underneath the namespace
// nmr for Numerics as in numeric computation.
//
// By Jesse Carpenter (carpentersoftware.com)
//
// Testing Platform:
//  * MCU:Atmega328P
//  * IDE:PlatformIO
//  * Editor: VSCode
//
// MIT LICENSE
//

#ifndef DDMR_L298N_h
#define DDMR_L298N_h

#include <Arduino.h>

namespace csm
{
    class L298N
    {
    private:
        uint8_t _ZERO;

        uint8_t _L298N_ENA;
        uint8_t _L298N_IN1;
        uint8_t _L298N_IN2;

        uint8_t _L298N_IN3;
        uint8_t _L298N_IN4;
        uint8_t _L298N_ENB;

        // Private method
        void _setDirectionPins(int const leftMotorPWM, int const rightMotorPWM);

    public:
        L298N();
        L298N(uint8_t const ENA,
              uint8_t const IN1,
              uint8_t const IN2,
              uint8_t const IN3,
              uint8_t const IN4,
              uint8_t const ENB);

        // Place inside setup() function
        // In the order as shown...
        void SetupPinsL298N();
        void PowerDownL298N();

        // Place inside loop() function
        void updateL298N(int left, int const right);
    };

    L298N::L298N()
    {
        _ZERO = 0;

        // Arduino Uno
        _L298N_ENA = 10; // PWM
        _L298N_IN1 = 9;  // Didital-Input
        _L298N_IN2 = 8;  // Didital-Input

        _L298N_IN3 = 7; // Didital-Input
        _L298N_IN4 = 6; // Didital-Input
        _L298N_ENB = 5; // PWM
    }

    L298N::L298N(uint8_t const ENA,
                 uint8_t const IN1,
                 uint8_t const IN2,
                 uint8_t const IN3,
                 uint8_t const IN4,
                 uint8_t const ENB)
    {

        _ZERO = 0;

        // NOT Arduino Uno
        _L298N_ENA = ENA; // PWM
        _L298N_IN1 = IN1; // Didital-Input
        _L298N_IN2 = IN2; // Didital-Input

        _L298N_IN3 = IN3; // Didital-Input
        _L298N_IN4 = IN4; // Didital-Input
        _L298N_ENB = ENB; // PWM
    }

    // Adjust  Motors
    void L298N::updateL298N(int left, int right)
    {
        // L298N_DirectionPins requires negative numbers
        _setDirectionPins(left, right);

        // L298N does not use negative values
        // If both Input Pair Pins (IN1 & IN2, IN3 & IN4) are
        // all set to LOW then the motors will not RUN..., no matter
        // what the motor speed values might be.
        // See L298N_DirectionPins() method for details.
        //
        // Left Motor
        analogWrite(_L298N_ENA, abs(left));
        // Right Motor
        analogWrite(_L298N_ENB, abs(right));
    }

    void L298N::PowerDownL298N()
    {
        digitalWrite(_L298N_IN1, LOW);
        digitalWrite(_L298N_IN2, LOW);
        digitalWrite(_L298N_IN3, LOW);
        digitalWrite(_L298N_IN4, LOW);
    }

    void L298N::SetupPinsL298N()
    {
        // INTERFACE: Motors
        pinMode(_L298N_ENA, OUTPUT);
        pinMode(_L298N_IN1, OUTPUT);
        pinMode(_L298N_IN2, OUTPUT);
        pinMode(_L298N_ENB, OUTPUT);
        pinMode(_L298N_IN3, OUTPUT);
        pinMode(_L298N_IN4, OUTPUT);
        //
        PowerDownL298N();
    }

    void L298N::_setDirectionPins(int const leftMotorPWM, int const rightMotorPWM)
    {
        // Center Tolerance is adjusted in Joystick Method
        // in the Manager Class (Header and Code Files)
        //
        // Note: IN1 and IN2 wheel direction can be changed
        //       by changing the conditionals "<" and ">"...,
        //       the order of the digitalWrite().
        //       Much easier changing code then wires on a motor.
        //
        // WARNING: Never set paired 1&2 values to HIGH at
        //          the same time. ALways set the FIRST-INPUT-PIN
        //          to LOW before setting the following PIN to HIGH.
        //          This will prevent two PINS from being HIGH at
        //          once.
        // This also includes IN3 and IN4 wheel direction pins below...

        // Left Motor //////////////////////////////////
        if (leftMotorPWM > _ZERO)
        {
            digitalWrite(_L298N_IN3, LOW);
            digitalWrite(_L298N_IN4, HIGH);
        }
        else if (leftMotorPWM < _ZERO)
        {
            digitalWrite(_L298N_IN4, LOW);
            digitalWrite(_L298N_IN3, HIGH);
        }
        else
        {
            digitalWrite(_L298N_IN3, LOW);
            digitalWrite(_L298N_IN4, LOW);
        }

        // Right Motor //////////////////////////////////
        if (rightMotorPWM > _ZERO)
        {
            digitalWrite(_L298N_IN1, LOW);
            digitalWrite(_L298N_IN2, HIGH);
        }
        else if (rightMotorPWM < _ZERO)
        {
            digitalWrite(_L298N_IN2, LOW);
            digitalWrite(_L298N_IN1, HIGH);
        }
        else
        {
            digitalWrite(_L298N_IN1, LOW);
            digitalWrite(_L298N_IN2, LOW);
        }
    }
}

#endif