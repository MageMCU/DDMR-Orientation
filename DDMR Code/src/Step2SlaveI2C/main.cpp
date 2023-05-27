//
// Carpenter Software
// File: SlaveUno: main.cpp
//
// Purpose: Public Github Account - MageMCU
// Repository: Communication
// Date Created: 20230425
// Folder: Digital Signals
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
#include "LinearMap.h"
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

// Global Declartions
dsg::BusI2C gBusI2C;
csm::Joystick<float> gJoystick;
nmr::LinearMap<float> gMapInput;
nmr::LinearMap<float> gMapOutput;
nmr::Timer gLoopTimer;

// Callback
void SlaveReceiveI2C(int numberBytes)
{
    uint8_t message[numberBytes];
    gBusI2C.ReceiveMessage(message, numberBytes);

    // Build the data back to its original values....
    if (message[0] == SLAVE_ADDR_0x16)
    {
        stateID = message[1];
        xSpeedInteger = gBusI2C.BytesToWord(message[2], message[3]);
        ySpeedInteger = gBusI2C.BytesToWord(message[4], message[5]);
    }
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
    }

    // Assign BusI2C Object
    gBusI2C = dsg::BusI2C();
    // Wire.BEGIN: Include Timeout
    gBusI2C.Begin(SLAVE_ADDR_0x16, 3000, true);

    // Assign Joystick Algorithm Object
    gJoystick = csm::Joystick<float>();

    // Conversion: integers to floats (setup)
    gMapInput = nmr::LinearMap<float>((float)0, (float)1023, (float)-1, (float)1);
    // Conversion: floats to integers (setup)
    gMapOutput = nmr::LinearMap<float>((float)-1, (float)1, (float)-255, (float)255);
}

void loop()
{
    if (gLoopTimer.isTimer(250))
    {
        // I2C RECEIVE
        Wire.onReceive(SlaveReceiveI2C);

        // Control integers
        Serial.print("s: ");
        Serial.print(stateID);
        Serial.print(" cX: ");
        Serial.print(xSpeedInteger);
        Serial.print(" cY: ");
        Serial.println(ySpeedInteger);

        // Convert Control values to Joystick Input
        xInput = gMapInput.Map((float)xSpeedInteger);
        yInput = gMapInput.Map((float)ySpeedInteger);

        // Input floats
        Serial.print(" iX: ");
        Serial.print(xInput);
        Serial.print(" iY: ");
        Serial.println(yInput);

        // Joystick - Input
        gJoystick.UpdateInputs(xInput, yInput);
        // Joystick - Output
        leftOuput = gJoystick.OutLeft();
        rightOutput = gJoystick.OutRight();

        // Output floats
        Serial.print(" oL: ");
        Serial.print(leftOuput);
        Serial.print(" oR: ");
        Serial.println(rightOutput);

        // Simulate L298N algorithm
        // integer has to be signed...
        leftMotor = (int)gMapOutput.Map(leftOuput);
        rightMotor = (int)gMapOutput.Map(rightOutput);

        // Motors before integers (L298N internal processing)
        Serial.print(" mbL: ");
        Serial.print(leftMotor);
        Serial.print(" mbR: ");
        Serial.println(rightMotor);

        // Motors after integers (L298N input)
        Serial.print(" maL: ");
        Serial.print(abs(leftMotor));
        Serial.print(" maR: ");
        Serial.println(abs(rightMotor));

        // L298N algorithm (L298N.h class) not yet incorporated...
    }

    // Prevent hiccups (stalls)
    gBusI2C.ClearTimeout();
}