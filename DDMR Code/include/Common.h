//
// Carpenter Software
// File: Common.h
//
// Purpose: Public Github Account - MageMCU
// Repository:
// Folder:
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

#ifndef CommonDDMR_h
#define CommonDDMR_h

//--------------------------------------
// MASTER UNO --------------------------
#define MASTER_ADDR_0x14 0x14
// SLAVE UNO----------------------------
#define SLAVE_ADDR_0x16 0x16
// MESSAGE SIZE
#define I2C_MESSAGE_SIZE 7

// USE A SINGLE WIRE AS A SWITCH
// DO NOT USE TACTILE BUTTON
#define LED_PIN 12
#define SWITCH_PIN 2

#define ON 1
#define OFF 0

//--------------------------------------

// COMMON Global Variables
// uint8_t stateID;

// loState uses 8-bits
// bit0 - used for switch (OFF ON)
// bit1 - used for turn (Left-CCW Right-CW)
// bit2 - not used
// bit3 - not used
// bit4 - not used
// bit5 - not used
// bit6 - not used
// bit7 - not used
uint8_t loState;

// hiState uses (uint8_t) integer
// 0 - Start State
// 1 - Enter Setpoint State
// 2 - Setpoint State
// 3 - Turn Direction State
// 4 - Orientation State
// 5 - not used
// 6 - not used
// 7 - not used
uint8_t hiState;

// Motors Speeds (16-bit word integers only)
// Use 10-bit digits (0 - 1023)
//   0 ---- 511 ---- 1023 (I2C data as 16-bit word)
//  -1 ----   0 ---- 1    (Joystick Algorithm floats)
// Once data is received by the Slave-MCU use
// mapping to convert 10-bit digits to floats...
int xSpeedInteger; // used for turn only
int ySpeedInteger; // used for forward only
// Both may be used together...

float xInput;
float yInput;

float leftOuput;
float rightOutput;

int leftMotor;
int rightMotor;

String PrintState(uint8_t state)
{
    String str = "";
    switch (state)
    {

    case 0:
        // str = "Start State";
        str = "ST"; // Use this for CSV
        break;

    case 1:
        // str = "Enter Setpoint State";
        str = "ES"; // Use this for CSV
        break;

    case 2:
        // str = "Setpoint State";
        str = "SS"; // Use this for CSV
        break;

    case 3:
        // str = "Turn Direction State";
        str = "TS"; // Use this for CSV
        break;

    case 4:
        // str = "Orientation State";
        str = "OS"; // Use this for CSV
        break;

    default:
        // Used for nullState
        str = "Print State at Default"; // DO NOT USE
        break;
    }
    return str;
}

#endif