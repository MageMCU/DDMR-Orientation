//
// Carpenter Software
// File: src: MAIN: main.cpp
//
// Purpose: Public Github Account - MageMCU
// Repository: ManagerFSM
// Date Created: 20230503
// Folder: OOP-ManagerFSM
//
// Code was adapted from Programming Game AI by Example (2005)
// by Mat Buckland...
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

#include "ManagerFSM.h"
#include "Timer.h"

using namespace fsm;
using namespace nmr;

// Manager inaccessible here -------------- Using Varient
// Debugging must occur inside manager...................
// STATES ---------------------------------------- STATES
// *A* IdleState - At startup, the State Machine intially
//         uses the Enter() method...
//     (1) Auto-Transition (non-conditional) to SetPointState 
// *B* SetPointState
//     (1) Choose a random angle (-180 to 180)
//         as the set-point (SP) and it ought
//         to be greater than the previous angle
//         by 30 degrees.
//     (2) Convert (SP) angle to vector...
//     (3) Auto-Transition (non-conditional) to OrientationState
// *C* OrientationState
//     (1) Magnetometer Reading angle as the
//         measured value (MV)... Note the
//         author here has several choices
//         among what magnetometer to use...
//         Decision made to choose the one with
//         the smallest code (Adafruit's code now
//         is way too large)... KEEP IT SIMPLE...
//     (2) Convert (MV) angle to vector...
//     (3) Decide Turning Direction (USE 2D-PerpDot)
//     (4) Use Dot-Product between Set-Point Vector
//         and Measured-Value Vector scalar result
//         as the PID-Error... (Use hand-wheel to
//         rotate compass while debugging the error)
//     (5) Simulate Motors (rotate robot until error
//         approach ~zero... Use the hand-wheel to
//         rotate compass while debugging the error...)
//         Include the turning direction: should be
//         the shortest distance around a unit-circle
//         either clockwise (CW) or counter-clockwise
//         (CCW).
//     (6) Flag-Transition (conditional) to IdleState *A* 
//         The flag is used until controll-error approaches
//         zero (or very close to zero)... Also this state
//         may be used as a safe state to stop or to continue
//         debugging other states... From here, will transition
//         back into IdleState-Enter()... See *A*...
// MOTORS ---------------------------------------- MOTORS
// ------------------------------------------------------ PENDING
//

// GLOBAL ---------------------------------------- FIXME
// fsm::ManagerFSM gManager;
ManagerFSM<float> gFSM;
Timer gTimerFSM;

void setup()
{
    // DEBUG
    Serial.begin(9600);
    while (!Serial)
    {
        /* code */
    } 
    // ManagerFSM
    gFSM = ManagerFSM<float>();
    gFSM.Begin();

}

void loop()
{
    if (gTimerFSM.isTimer(100))
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

        gFSM.Update();

        // DeltaTime (how much time does gFSM.Update(); uses?)
        // Serial.print("dT: ");
        // Serial.println(gTimerFSM.DeltaTimeSeconds());
    }
}