//
// Carpenter Software
// Folders: src: Step2MasterI2C:
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
    // States
    message[1] = loState;
    message[2] = hiState;
    // Input-X
    gBusI2C.WordToBytes(xSpeedInteger);
    message[3] = gBusI2C.GetHiByte();
    message[4] = gBusI2C.GetLoByte();
    // Input-Y
    gBusI2C.WordToBytes(ySpeedInteger);
    message[5] = gBusI2C.GetHiByte();
    message[6] = gBusI2C.GetLoByte();
    // TRANSMIT
    gBusI2C.TransmitMessage(SLAVE_ADDR_0x16, message, I2C_MESSAGE_SIZE);
    // Print all errors...
    // Serial.println(gBusI2C.ErrorMsg());
}

void TransmitControlData()
{
    xSpeedInteger = 511;
    ySpeedInteger = 511;

    // if switch is ON
    if (loState > (uint8_t) 0)
    {
        xSpeedInteger = gCMV2.SpeedIntegerX();
        ySpeedInteger = gCMV2.SpeedIntegerY();
    }

    // Debug
    // Serial.print("s: ");
    // Serial.print(gState);
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
        // Control Update compass heading
        gCMV2.Update(gCompass.Update(), false);
        // Current State
        hiState = gCMV2.CurrentState();

        // CAUTION:
        // SEE Switch.h FOR INSTRUCTIONS
        // Button.h - Not Used...
        if (gSwitch.isSwitchOn())
        {
            // Bit Number Zero
            // SLAVE Motors - ON
            loState = (uint8_t) ON;
        }
        else
        {
            // Bit Number Zero
            // Pull-Wire To Stop SLAVE Motors - OFF;
            loState = (uint8_t) OFF;
        }

        // Bit Number One (additive)
        // Left (CCW) Turn Direction
        if (gCMV2.IsDirectionCCW())
            loState += (uint8_t) 2;

        // I2C TRANSMIT
        TransmitControlData();

        // Debug
        // Serial.print("lo: ");
        // Serial.print(loState);
        // Serial.print(" hi: ");
        // Serial.println(hiState);

        // DeltaTime (how much time does gCMV2.Update(); uses?)
        // Serial.print("dT: ");
        // Serial.println(gTimerFSM.DeltaTimeSeconds());
    }

    // Prevent hiccups (stalls)
    // Includes Compass code
    // Includes Master-Slave code
    gBusI2C.ClearTimeout();
}