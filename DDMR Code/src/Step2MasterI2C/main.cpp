//
// Carpenter Software
// Folders: src: Step1STATES: DotProduct:
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

#include "ControlManagerV2.h"
#include "BusI2C.h"
#include "Common.h"
#include "Compass.h"
#include "Timer.h"
#include "Switch.h"

using namespace dsg;
using namespace csm;
using namespace nmr;
using namespace uno;

// GLOBAL
ControlManagerV2<float> gCMV2;
BusI2C gBusI2C;
Compass<float> gCompass;
Timer gTimerFSM;
Switch gSwitch;

void setup()
{
    // DEBUG
    Serial.begin(9600);
    while (!Serial)
    {
        /* code */
    }

    // ControlManagerV2
    gCMV2 = ControlManagerV2<float>();

    // Bus
    gBusI2C = BusI2C();
    gBusI2C.Begin(MASTER_ADDR_0x14, 3000, true);
    delay(6);

    // Compass
    gCompass = Compass<float>();
    gCompass.Begin();
    delay(6);

    // Switch uses:
    // m_switchPin 2
    // m_ledPin 12
    gSwitch = Switch();
}

void i2cTransmitToSlave()
{
    // I2C Message Array Byte Size
    uint8_t message[I2C_MESSAGE_SIZE];
    // Arrange Bytes
    message[0] = (uint8_t)SLAVE_ADDR_0x16;
    message[1] = (uint8_t)stateID;
    // Input-X
    gBusI2C.WordToBytes(xSpeedInteger);
    message[2] = gBusI2C.GetHiByte();
    message[3] = gBusI2C.GetLoByte();
    // Input-Y
    gBusI2C.WordToBytes(ySpeedInteger);
    message[4] = gBusI2C.GetHiByte();
    message[5] = gBusI2C.GetLoByte();
    // TRANSMIT
    gBusI2C.TransmitMessage(SLAVE_ADDR_0x16, message, I2C_MESSAGE_SIZE);
    // Print all errors...
    // Serial.println(gBusI2C.ErrorMsg());
}

void TransmitControlData()
{
    xSpeedInteger = 511;
    ySpeedInteger = 511;
    
    if (stateID == (uint8_t)ON)
    {
        xSpeedInteger = gCMV2.SpeedIntegerX();
        ySpeedInteger = gCMV2.SpeedIntegerY();
    }

    // Debug
    // Serial.print("s: ");
    // Serial.print(stateID);
    // Serial.print(" x: ");
    // Serial.print(xSpeedInteger);
    // Serial.print(" y: ");
    // Serial.print(ySpeedInteger);
    // Serial.print(" ");


    // Send data to Slave
    i2cTransmitToSlave();
}

void loop()
{
    gSwitch.updateSwitch();
    // BEWARE: PID Sampled-Time has to be the same
    // as the timer - see ControlManagerV2
    // constructor...
    // Note: 1000ms = 1s
    if (gTimerFSM.isTimer(250))
    {
        // CAUTION:
        // SEE Switch.h FOR INSTRUCTIONS
        // Button.h - Not Used...
        if (gSwitch.isSwitchOn())
        {
            stateID = (uint8_t)ON;

            // Control Update compass heading
            gCMV2.Update(gCompass.Update(), false);
        }
        else
        {
            // Pull-Wire To Stop Motors
            stateID = (uint8_t)OFF;
        }

        // I2C TRANSMIT
        TransmitControlData();

        // DeltaTime (how much time does gCMV2.Update(); uses?)
        // Serial.print("dT: ");
        // Serial.println(gTimerFSM.DeltaTimeSeconds());
    }

    // Prevent hiccups (stalls)
    // Includes Compass code
    // Includes Master-Slave code
    gBusI2C.ClearTimeout();
}