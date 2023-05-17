//
// Carpenter Software
// File: Class BusI2C.h
//
// Purpose: Public Github Account - MageMCU
// Repository: Communication
// Date Created: 20230219
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

#ifndef BUS_I2C_h
#define BUS_I2C_h

// IN DEVELOMENT ------------------------------- FIXME
namespace dsg
{
    class BusI2C
    {
    public:
        // Public Properties
        // Constructor
        BusI2C();
        ~BusI2C() = default;

        // Public Methods
        void Begin(uint8_t address);
        void Begin(uint8_t address, uint32_t timeout, bool reset);
        void ClearTimeout();
        String IsDevice(uint8_t deviceAddress);
        String ErrorMsg();
        uint8_t GetErrorNumberI2C();
        void ReceiveMessage(uint8_t *pMessage, uint8_t size);
        uint8_t RequestMessage(uint8_t addr, uint8_t msg[], uint8_t size);
        void TransmitMessage(uint8_t addr, uint8_t msg[], uint8_t size);
        uint8_t RecieveDeviceMessage(uint8_t addr, uint8_t reg, uint8_t msg[], uint8_t size);
        void WordToBytes(uint16_t word);
        uint8_t GetHiByte();
        uint8_t GetLoByte();
        uint16_t BytesToWord();
        uint16_t BytesToWord(uint8_t hiByte, uint8_t loByte);

    private:
        // Private Properties
        uint8_t m_errorI2C;
        // AVR-LSB Word Order bit15, bit14, ... , bit1, bit0.
        uint16_t b_wordIN;
        uint16_t b_wordOUT;
        // AVR-LSB Byte Order bit7, bit6, ... , bit1, bit0.
        uint8_t b_byteHi;
        uint8_t b_byteLo;

        // Private Methods
        String m_errorMessageI2C();
        void m_scanningI2C(uint8_t deviceAddress);
        void b_setHiByte();
        void b_setLoByte();
        void b_glueBytes();
    };

}
#endif
