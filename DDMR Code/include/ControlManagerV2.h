//
// Carpenter Software
// File: Class ControlManagerV2.h
// Usage: (1) Vector2 (direction vectors)
//        (2) Perpendicular-Dot Product
//            for turning-direction
//        (3) Dot Product (PID error control)
//            for PID error control...
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

#ifndef Control_Manager_V2_h
#define Control_Manager_V2_h

#include <Arduino.h>
#include <Wire.h>

#include "BusI2C.h"
#include "LSM303.h"
#include "Vector2.h"
#include "Controller.h"
#include "Statistics.h"

using namespace dsg;
using namespace nmr;
using namespace pid;

// MASTER UNO BOARD
#define MASTER_ADDR_0x14 0x14

// SLAVE Atmega328P Breadboard
// Used for motors....
// #define SLAVE_ADDR_0x16 0x16

// TUPLES_SIZE
// Sould be less than looptimer cycles
// which is 10 cycles per secend (100ms)
#define TUPLES_SIZE 5

namespace fsm
{
    // Priority (inside states use if-else)
    enum States
    {
        // States < 10
        StartState = 1,         // Rename StartState
        SetpointState = 2,      // OK
        TurnDirectionState = 3, // OK
        OrientationState = 4,   // OK
                                // New States
        MotorsTurnState = 5,    // OK
        MotorsForwardState = 6, // OK

        // Enter States >= 10
        EnterSetpointState = 12,      // OK
        EnterTurnDirectionState = 13, // pending removal
        EnterOrientationState = 14,   // pending removal
                                      // New States
        EnterMotorsTurnState = 15,    // pending removal
        EnterMotorsForwardState = 16, // pending removal

        // Exit States >= 20 NONE

        // NULL STATE
        nullState = -1
    };

    template <typename real>
    class ControlManagerV2
    {
    private:
        // Objects
        BusI2C m_busI2C;
        LSM303 m_compass;
        Controller<real> m_pid;
        Statistics<real> m_stats;

        // Private Properties

        // FSM
        States m_currentState;
        States m_lastState;

        // Direction Vector Set Point
        real m_angleSP; // Random Value
        Vector2<real> m_vSP;

        // Direction Vector Measured Value
        real m_angleMV; // Compass
        Vector2<real> m_vMV;

        // PID experimental
        // CCW: left
        bool m_directionCCW;
        // Using sign to determine turn direction.
        real m_perpDot;
        // Dot Product under study
        real m_dot;
        // Dot Angle for uf
        real m_dotAngle;
        // Anglular velocity
        real m_lastDotAngle;
        real m_angularVelocity;
        // control function
        real m_uf;

        // Statistics
        // real m_tuples[TUPLES_SIZE];
        // int m_index;
        // real m_mean;
        // real m_sd;

        // Motors
        real m_xMotors; // used for turn only
        real m_yMotors; // used for forward only
        // Both may be used...

        // Private Setters
        void m_setAngleSP(real angle);
        void m_setAngleMV(real angle);

        // Private Methods
        void m_randomAngle();
        void m_angleToVectorSP();
        void m_angleToVectorMV();
        real m_sign(real value);

        // Private States
        bool m_startState();
        bool m_enterSetpointState();
        bool m_setpointState();
        bool m_enterTurnDirectionState();
        bool m_turnDirectionState();
        bool m_enterOrientationState();
        bool m_orientationState();
        // New State Methods
        bool m_enterMotorsTurnState();
        bool m_motorsTurnState();
        bool m_enterMotorsForwardState();
        bool m_motorsForwardState();

        // Private Supporting Update Methods
        Vector2<real> m_directionVector(real radian);
        void m_updateCompass();
        void m_updatePID();
        bool m_turnChanged();
        bool m_controlPass();
        bool m_stateChanged();
        // real m_statistics(real value);
        void m_updateFSM();

        // Debug
        void m_debugPlotCSV();
        String m_printState();

    public:
        // Constructor - inline
        ControlManagerV2()
        {
            m_angleSP = (real)0;
            m_angleMV = (real)0;
            // Experimental (See updatePID())
            m_dotAngle = (real)0;
            m_lastDotAngle = (real)0;
            m_angularVelocity = (real)0;

            // I2C Bus
            m_busI2C = BusI2C();

            // Compass
            m_compass = LSM303();

            // Current State ID
            m_currentState = States::StartState;
            m_lastState = States::nullState;

            // PID Controller
            real Kp = (real)0.95;
            real Ki = (real)0.0;
            real Kd = (real)0.0;
            // BEWARE: Sampled-Interval Ts
            // has to have the same time
            // interval as loop() timer...
            real Ts = (real)0.1; // See m_updatePID()
            m_pid = Controller<real>(Kp, Ki, Kd, Ts);
            // Assumming error approaches zero...
            m_pid.SetPoint((real)0);
            m_uf = (real)0;

            // Motors

            // Statistics (optional)
            // Tuples used to get Mean & SD...
            // Used inside m_updateCompass()
            // for (int i = 0; i < TUPLES_SIZE; i++)
            //     m_tuples[i] = (real)0;
            // m_stats = nmr::Statistics<real>(m_tuples, TUPLES_SIZE);
            // // Index counter for Queue()
            // m_index = 0;
        }

        ~ControlManagerV2() = default;

        // ControlManagerV2 setup()
        void Begin();
        /////////////////////////////////////////////////////////////
        // ControlManagerV2 loop() inline
        void Update()
        {
            // Heading
            m_updateCompass();

            // PID
            m_updatePID();

            // FSM where (m_lastState) used in m_stateChanged()
            m_updateFSM();
            // Call m_updateFSM() before assigning m_lastState...
            m_lastState = m_currentState;

            // CSV - Debug Comment this or the other....
            m_debugPlotCSV();
        }
        /////////////////////////////////////////////////////////////
        // Getters
        real GetAngleSP();
        real GetAngleMV();
        real MotorsX();
        real MotorsY();
    };

    template <typename real>
    void ControlManagerV2<real>::Begin()
    {
        // Wire startup
        m_busI2C.Begin(MASTER_ADDR_0x14, 3000, true);

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

    // PUBLIC GETTERS

    template <typename real>
    real ControlManagerV2<real>::GetAngleSP()
    {
        return m_angleSP;
    }

    template <typename real>
    real ControlManagerV2<real>::GetAngleMV()
    {
        return m_angleMV;
    }

    template <typename real>
    real ControlManagerV2<real>::MotorsX()
    {
        return m_xMotors;
    }

    template <typename real>
    real ControlManagerV2<real>::MotorsY()
    {
        return m_yMotors;
    }

    // PRIVATE SETTERS

    template <typename real>
    void ControlManagerV2<real>::m_setAngleSP(real angle)
    {
        m_angleSP = angle;
    }

    template <typename real>
    void ControlManagerV2<real>::m_setAngleMV(real angle)
    {
        m_angleMV = angle;
    }

    // PRIVATE METHODS

    template <typename real>
    void ControlManagerV2<real>::m_randomAngle()
    {
        real angle;
        real delta;

        do
        {
            // Creates Random Angle from 0 to 360
            angle = (real)(rand() % 360);
            delta = abs(angle - m_angleSP);
            // Caught Bug by using dots
            // Serial.print(".");
        } while (delta < (real)30);
        m_angleSP = angle;

        // Convert SP angle to vector
        m_angleToVectorSP();
    }

    template <typename real>
    void ControlManagerV2<real>::m_angleToVectorSP()
    {
        real rad = GetAngleSP() * DEG_TO_RAD;
        m_vSP = m_directionVector(rad);
    }

    template <typename real>
    void ControlManagerV2<real>::m_angleToVectorMV()
    {
        real rad = GetAngleMV() * (real)DEG_TO_RAD;
        m_vMV = m_directionVector(rad);
    }

    template <typename real>
    real ControlManagerV2<real>::m_sign(real value)
    {
        real sign = (real)1;
        if (value < 0)
            sign = (real)-1;
        return sign;
    }
    // PRIVATE METHODS

    template <typename real>
    Vector2<real> ControlManagerV2<real>::m_directionVector(real radian)
    {
        // MiscMath to Vector2
        real a = cos(radian);
        real b = sin(radian);
        Vector2<real> vector(a, b);
        return vector;
    }

    template <typename real>
    void ControlManagerV2<real>::m_updateCompass()
    {
        // Compass Update LSM303
        m_compass.read();

        // Compass Heading
        real angle = m_compass.heading();

        // Declination
        real declinationAngle = (8.0 + (3.0 / 60.0));
        angle += declinationAngle;

        // Keep angle between 0 - 360
        if (angle < (real)0)
            angle = angle + (real)360;
        if (angle > (real)360)
            angle = angle - (real)360;

        // OPTIONAL (can be commented out)
        // angle = m_statistics(angle);

        // Set Measured Value
        m_setAngleMV(angle);

        // Require both Vector SP and Vector MV
        m_angleToVectorMV();

        // Prevent hiccups (stalls)
        m_busI2C.ClearTimeout();
    }

    template <typename real>
    void ControlManagerV2<real>::m_updatePID()
    {
        real processPoint;
        // Determine Turning Direction
        // Angle range for both are 0 to 360 degrees...
        m_perpDot = m_vMV.PerpDot(m_vSP);
        m_dot = m_vMV.Dot(m_vSP);
        m_dotAngle = m_vMV.Angle(m_vSP) * RAD_TO_DEG;
        // PID Setpoint
        // Setpoint was assigned within the constructor (zero)...
        // PID Process Point
        processPoint = m_dotAngle * m_sign(m_perpDot);
        // Kp = 0.95, Ki = 0, Kd = 0 (see constructor)
        m_pid.UpdatePID(processPoint);
        // PID Control Function
        m_uf = m_pid.Control();
    }

    template <typename real>
    bool ControlManagerV2<real>::m_turnChanged()
    {
        bool turnChanged = false;
        if (m_directionCCW)
        {
            turnChanged = m_perpDot >= (real)0;
        }
        else
        {
            turnChanged = m_perpDot < (real)0;
        }

        return turnChanged;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_controlPass()
    {
        // Used along with m_turnChanged()
        if (m_uf < (real)10)
            return true;
        return false;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_stateChanged()
    {
        if ((int)m_currentState != (int)m_lastState)
            return true;
        return false;
    }

    // template <typename real>
    // real ControlManagerV2<real>::m_statistics(real value)
    // {
    //     // Statistics (5-point data queue)
    //     // See ControlManagerV2 CLASS
    //     if (m_index >= TUPLES_SIZE)
    //         m_index = 0;
    //     m_stats.Queue(value, m_index);
    //     m_mean = m_stats.Average();
    //     m_sd = m_stats.StandardDeviation();
    //     m_index++;
    //     return m_mean;
    // }

    // STATES

    template <typename real>
    bool ControlManagerV2<real>::m_startState()
    {
        // Nothing to do here...

        // Prepare FSM to Transition To Setpoint State
        m_currentState = States::EnterSetpointState;

        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_enterSetpointState()
    {
        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_setpointState()
    {
        // OK to use here
        // Choose a random angle (-180 to 180) as the set-point (SP)
        // and greater than the previous angle by 30 degrees.
        m_randomAngle();

        // Prepare FSM to Transition To Turn Direction State
        m_currentState = States::EnterTurnDirectionState;

        // Ready Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_enterTurnDirectionState()
    {
        // Ready Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_turnDirectionState()
    {
        // OK to use variable here...
        if (m_perpDot < 0)
            m_directionCCW = true; // Left-turn
        else
            m_directionCCW = false; // Right-turn

        m_currentState = States::EnterOrientationState;
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_enterOrientationState()
    {
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_orientationState()
    {
        bool transitionflag = false;
        bool turnflag = m_turnChanged();
        // turn direction will constantly switch
        // back and forth until m_uf is low enough...
        // Units Degrees
        bool spFlag = abs(m_uf) < (real)2.5;
        bool xMotorsFlag = abs(m_uf) > (real)15; // 4x spFlag

        // Priority 0 - Not yet implemented
        if (xMotorsFlag)
        {
            // Turn only
            if (m_directionCCW)
                m_xMotors = (real)0.75; // CCW
            else
                m_xMotors = (real)-0.75; // CW

            // Stop yMotors
            m_yMotors = (real)0;
        }
        else
        {
            // Turn slowly
            if (m_directionCCW)
                m_xMotors = (real)0.25; // CCW
            else
                m_xMotors = (real)-0.25; // CW

            // Move forward
            m_yMotors = (real)7.5;
        }

        if (spFlag && turnflag)
        {
            // Priority 1 - Good
            m_currentState = States::EnterSetpointState;
            transitionflag = true;
        }
        else if (turnflag)
        {
            // Priority 2 - Good
            m_currentState = States::EnterTurnDirectionState;
            transitionflag = true;
        }

        // Flagged Transition
        return transitionflag;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_enterMotorsTurnState()
    {
        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_motorsTurnState()
    {
        // Nothing to do here...

        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_enterMotorsForwardState()
    {
        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_motorsForwardState()
    {
        // Nothing to do here...

        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    void ControlManagerV2<real>::m_updateFSM()
    {
        // Finite State Machine
        switch (m_currentState)
        {
        case States::StartState:
            // Must have a State Method
            if (m_startState())
            {
                // Transition To...
                m_currentState = States::EnterSetpointState;
            }
            break;

        case States::EnterSetpointState:
            // Must have a State Method
            if (m_enterSetpointState())
            {
                // Transition To...
                m_currentState = States::SetpointState;
            }
            break;

        case States::SetpointState:
            // Must have a State Method
            if (m_setpointState())
            { /* Allow State to Decide */
            }
            break;

        case States::EnterTurnDirectionState:
            // Must have a State Method
            if (m_enterTurnDirectionState())
            {
                // Transition To...
                m_currentState = States::TurnDirectionState;
            }
            break;

        case States::TurnDirectionState:
            // Must have a State Method
            if (m_turnDirectionState())
            { /* Allow State to Decide */
            }
            break;

        case States::EnterOrientationState:
            // Must have a State Method
            if (m_enterOrientationState())
            {
                // Transition To...
                m_currentState = States::OrientationState;
            }
            break;

        case States::OrientationState:
            // Must have a State Method
            if (m_orientationState())
            { /* Allow State to Decide */
            }
            break;

        case States::EnterMotorsTurnState:
            // Must have a State Method
            if (m_enterOrientationState())
            {
                // Transition To...
                m_currentState = States::MotorsTurnState;
            }
            break;

        case States::MotorsTurnState:
            // Must have a State Method
            if (m_orientationState())
            { /* Allow State to Decide */
            }
            break;

        default:
            break;
        }
    }

    template <typename real>
    void ControlManagerV2<real>::m_debugPlotCSV()
    {
        Serial.print(m_printState());
        Serial.print(",");

        if (m_directionCCW)
            Serial.print("CCW");
        else
            Serial.print("CW");

        Serial.print(",");
        Serial.print(GetAngleSP());
        Serial.print(",");
        Serial.print(GetAngleMV());
        Serial.print(",");
        Serial.print(m_uf);
        Serial.print(",");

        if (m_currentState == States::OrientationState)
            Serial.print("Turn");
        else
            Serial.print("Wait");

        Serial.println(",");
    }

    template <typename real>
    String ControlManagerV2<real>::m_printState()
    {
        String str = "";

        // Shoud match updateFSM()
        switch (m_currentState)
        {

        case States::StartState:
            // str = "Start State";
            str = "ST"; // Use this for CSV
            break;

        case States::EnterSetpointState:
            // str = "Enter Setpoint State";
            str = "ES"; // Use this for CSV
            break;

        case States::SetpointState:
            // str = "Setpoint State";
            str = "SS"; // Use this for CSV
            break;

        case States::EnterTurnDirectionState:
            // str = "Enter Turn Direction State";
            str = "ET"; // Use this for CSV
            break;

        case States::TurnDirectionState:
            // str = "Turn Direction State";
            str = "TS"; // Use this for CSV
            break;

        case States::EnterOrientationState:
            // str = "Enter Orientation State";
            str = "EO"; // Use this for CSV
            break;

        case States::OrientationState:
            // str = "Orientation State";
            str = "OS"; // Use this for CSV
            break;

        case States::EnterMotorsTurnState:
            // str = "Enter Motors Turn State";
            str = "EM"; // Use this for CSV
            break;

        case States::MotorsTurnState:
            // str = "Motors Turn State";
            str = "MT"; // Use this for CSV
            break;

        case States::EnterMotorsForwardState:
            // str = "Enter Motors Forward State";
            str = "MF"; // Use this for CSV
            break;

        case States::MotorsForwardState:
            // str = "Motors Forward State";
            str = "FS"; // Use this for CSV
            break;

        default:
            // Used for nullState
            str = "Print State at Default"; // Use this for CSV
            break;
        }
        return str;
    }
}

#endif
