//
// Carpenter Software
// File: Class Compass.h
// Usage: (1) Vector2 (direction vectors)
//        (2) Perpendicular-Dot Product
//            for turning-direction
//        (3) Angle Dot Product used for PID control function (uf)
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

#ifndef Compass_h
#define Compass_h

#include <Arduino.h>
#include <Wire.h>

#include "LSM303.h"

namespace dsg
{
    template<typename real>
    class Compass
    {
    private:
        // Objects
        LSM303 m_compass;

        // Private Properties
        real m_heading;

        // Private Methods
        void m_updateCompass();

    public:
        //Constructor
        Compass();
        ~Compass() = default;

        // Methods
        void Begin();
        real Update();
    };

    template<typename real>
    Compass<real>::Compass()
    {
        // Compass
        m_compass = LSM303();
    }

    template<typename real>
    void Compass<real>::Begin()
    {
        // Compass
        m_compass.init();
        m_compass.enableDefault();
        // Calibration values; the default values of +/-32767 for each axis...
        // Use Calibration Code
        // WAS: m_compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
        // WAS: m_compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
        m_compass.m_min = (LSM303::vector<int16_t>){-2747, -2711, -1975};
        m_compass.m_max = (LSM303::vector<int16_t>){+2978, +2856, +2723};
    }

    template<typename real>
    real Compass<real>::Update()
    {
        m_updateCompass();
        return m_heading;
    }

    template <typename real>
    void Compass<real>::m_updateCompass()
    {
        // Compass Update LSM303
        m_compass.read();

        // Compass Heading
        real heading = m_compass.heading();

        // Declination (under study)
        real declinationAngle = (8.0 + (3.0 / 60.0));
        heading += declinationAngle;

        // Keep angle between 0 - 360
        if (heading < (real)0)
            heading = heading + (real)360;
        if (heading > (real)360)
            heading = heading - (real)360;

        // Save Heading
        m_heading = heading;
    }
}

#endif
