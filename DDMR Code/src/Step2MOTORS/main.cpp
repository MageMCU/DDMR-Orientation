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
#include "ManagerNames.h"
#include "Timer.h"
// Manager inaccessible here -------------- Using Varient
// Debugging must occur inside manager...................
// STATES
// *A* IdleState - Begins without Enter()
//     (1) Auto-Transition to SetPointState
// *B* SetPointState
//     (1) Choose a random angle (-180 to 180)
//         as the set-point (SP) and it ought 
//         to be greater than the previous angle
//         by 30 degrees.
//     (2) Convert (SP) angle to vector...
//     (3) Auto-Transition to OrientationState
// *C* OrientationState
//     (1) Magnetometer Reading angle as the
//         measured value (MV)
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
//     (6) Flag-Transition to IdleState *A* This state
//         may be used as safe way to stop or to continue
//         debugging other states... 
// MOTORS
//

fsm::ManagerFSM manager;
nmr::Timer fsmTimer;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        /* code */
    }
    manager = fsm::ManagerFSM(manager_fsm);
}

void loop()
{
    if (fsmTimer.isTimer(1000))
    {
        manager.Update();
    }
}