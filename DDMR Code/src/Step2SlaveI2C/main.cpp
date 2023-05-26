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
        // Not receiving xCOntrol
        xSpeedInteger = gBusI2C.BytesToWord(message[2], message[3]); // FIXME
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
}

void loop()
{
    if (gLoopTimer.isTimer(250))
    {
        // I2C RECEIVE
        Wire.onReceive(SlaveReceiveI2C);

        // Algorithn Pending ------------------------------- FIXME
        
        // Debug
        Serial.print("s: ");
        Serial.print(stateID);
        Serial.print(" x: ");
        Serial.print(xSpeedInteger);
        Serial.print(" y: ");
        Serial.println(ySpeedInteger);
    }

    // Prevent hiccups (stalls)
    gBusI2C.ClearTimeout();
}