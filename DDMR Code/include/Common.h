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
#define I2C_MESSAGE_SIZE 6

// USE A SINGLE WIRE AS A SWITCH
// DO NOT USE TACTILE BUTTON
#define LED_PIN 12
#define SWITCH_PIN 2

#define ON 1
#define OFF 0

//--------------------------------------

// COMMON Global Variables
uint8_t stateID;

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

#endif