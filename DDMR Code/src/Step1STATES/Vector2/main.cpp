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

using namespace dsg; // digital signals
using namespace csm; // control state machine
using namespace nmr; // numerics

// GLOBALS
ControlManagerV2<float> gCMV2;
BusI2C gBusI2C;
Compass<float> gCompass;
Timer gTimerFSM;

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
}

void loop()
{
    if (gTimerFSM.isTimer(1000))
    {
        // (1) When using debug version, set
        // timer to 1000 ms...
        // (2) When using release version, the
        // statistics at 5-data points needs
        // the timer set to 200 ms for a 1-sec giving
        // 5-cycles per sec intervals for the mean (average)
        // and standard deviation... BEWARE see (3)
        // (I tried 100 ms timer without any issues...)
        // (3) The time sample (TS) for the PID has to be
        // a constant so do not allow algorithms
        // to overtake the timer... Place
        // Timer.DeltaTimeSeconds() at the end of loop
        // block within timer's loop... See below. Should
        // match the same time intervals for Timer.IsTimer()...
        // (4) Wire now uses a timeout where
        // here 3000us or 3ms is used... otherwise
        // the I2C bus will hangup basically saying
        // I give up....

        gCMV2.Update(gCompass.Update(), true);

        // DeltaTime (how much time does gCMV2.Update(); uses?)
        // Serial.print("dT: ");
        // Serial.println(gTimerFSM.DeltaTimeSeconds());
    }
}