//
// Carpenter Software
// Folders: src: Step2SlaveI2C:
// File: main.cpp
//
// Purpose: Public Github Account - MageMCU
// Repository: DDMR-Orientation
// Folder: DDMR Code
//
// Author: Jesse Carpenter (carpentersoftware.com)
// Email:carpenterhesse@gmail.com
//
// Testing Platform:
//  * MCU:Atmega328P
//  * Editor: VSCode
//  * VSCode Extension: Microsoft C/C++ IntelliSense, debugging, and code browsing.
//  * VSCode Extension:PlatformIO
//
// MIT LICENSE
//

#include <Arduino.h>
#include <Wire.h>

#include "Common.h"
#include "BusI2C.h"
#include "Joystick.h"
#include "L298N.h"
#include "LinearMap.h"
#include "Bitwise.h"
#include "Timer.h"

// Arduino Uno as Master communicates with another Arduino as Slave.
// ----------------------------------------------------
// CAUTION: platformio.ini ---------------------------- BEWARE
// MASTER MCU: Change to MasterUno folder
//             Upload to Master-MCU
// SLAVE MCU: Change to SlaveUno folder
//             Upload to Slave-MCU
// ----------------------------------------------------
// ----------------------------------------------------
// Note that some Wire call functions reside in both
// BusI2C.h and main.cpp...
// ----------------------------------------------------

using namespace dsg;
using namespace csm;
using namespace nmr;

// Global Declartions
BusI2C gBusI2C;
Joystick<float> gJoystick;
L298N gSingleMotor;
LinearMap<float> gMapInput;
LinearMap<float> gMapOutput;
Bitwise<uint8_t> gState;
Timer gLoopTimer;

// Callback
void SlaveReceiveI2C(int numberBytes)
{
    uint8_t message[numberBytes];
    gBusI2C.ReceiveMessage(message, numberBytes);

    // Build the data back to its original values....
    if (message[0] == SLAVE_ADDR_0x16)
    {
        loState = message[1];
        hiState = message[2];
        xSpeedInteger = gBusI2C.BytesToWord(message[3], message[4]);
        ySpeedInteger = gBusI2C.BytesToWord(message[5], message[6]);
    }
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
    }

    // Assign BusI2C Object
    gBusI2C = BusI2C();
    // Wire.BEGIN: Include Timeout
    gBusI2C.Begin(SLAVE_ADDR_0x16, 3000, true);

    // Assign Joystick Algorithm Object
    gJoystick = Joystick<float>();

    // Temp Variables
    int8_t ENA = 5;
    int8_t IN1 = 6;
    int8_t IN2 = 7;
    int8_t IN3 = 8;
    int8_t IN4 = 9;
    int8_t ENB = 10;
    int8_t LeftMotorPWM = ENB;
    int8_t LeftMotorIN1 = IN4;
    int8_t LeftMotorIN2 = IN3;
    int8_t RightMotorIN1 = IN1;
    int8_t RightMotorIN2 = IN2;
    int8_t RightMotorPWM = ENA;

    // Assign L298N Object
    gSingleMotor = L298N(LeftMotorPWM, LeftMotorIN1, LeftMotorIN2,
                         RightMotorIN1, RightMotorIN2, RightMotorPWM);
    gSingleMotor.SetupPinsL298N();

    // Conversion: integers to floats (setup)
    gMapInput = LinearMap<float>((float)0, (float)1023, (float)-1, (float)1);
    // Conversion: floats to integers (setup)
    gMapOutput = LinearMap<float>((float)-1, (float)1, (float)-255, (float)255);

    // Bitwise
    gState = Bitwise<uint8_t>();
}

void loop()
{
    if (gLoopTimer.isTimer(250))
    {
        // I2C RECEIVE
        Wire.onReceive(SlaveReceiveI2C);

        // Bit-0 - Essential for safety motors shutdown....
        bool stateON = false;
        if (gState.IsBitNumberSetToBitsValue((uint8_t)0, loState))
        {
            stateON = true;
        }

        // Bit-1 - Non-essentail debug only
        bool stateLeftTurn = false;
        if (gState.IsBitNumberSetToBitsValue((uint8_t)1, loState))
        {
            stateLeftTurn = true;
        }

        // Convert Control values to Joystick Input
        xInput = gMapInput.Map((float)xSpeedInteger);
        yInput = gMapInput.Map((float)ySpeedInteger);

        // Update Joystick Input
        gJoystick.UpdateInputs(xInput, yInput);
        // Joystick - Output
        leftOuput = gJoystick.OutLeft();
        rightOutput = gJoystick.OutRight();

        // For conversion values, see setup()...
        leftMotor = (int)gMapOutput.Map(leftOuput);
        rightMotor = (int)gMapOutput.Map(rightOutput);

        // SAFETY - MOTORS
        if (stateON)
        {
            // L298N
            gSingleMotor.updateL298N(leftMotor, rightMotor);
            // Debug - State Motor ON
            Serial.print("ON: ");
        }
        else
        {
            gSingleMotor.PowerDownL298N();
            // Debug - State Motor OFF
            Serial.print("OFF: ");
        }

        // Debug
        Serial.print(PrintState(hiState));

        // Debug
        if (stateLeftTurn)
            Serial.println(" CCW");
        else
            Serial.println(" CW");
    }

    // Prevent hiccups (stalls)
    gBusI2C.ClearTimeout();
}