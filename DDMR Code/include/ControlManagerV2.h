//
// Carpenter Software
// File: Class ControlManagerV2.h
// Usage: (1) Vector2 (direction vectors)
//        (2) Perpendicular-Dot Product
//            for turning-direction
//        (3) Angle Dot Product used for PID 
//            control function (uf) - here
//            degrees are used but may be
//            changed to radian-measure...
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

#include "Vector2.h"
#include "Controller.h"

using namespace nmr;
using namespace pid;

namespace csm // control state machine
{
    // Priority (inside states use if-else)
    enum States
    {
        // States < 10
        StartState = 1,         // Rename StartState
        SetpointState = 2,      // OK
        TurnDirectionState = 3, // OK
        OrientationState = 4,   // OK

        // Enter States >= 10
        EnterSetpointState = 12, // OK

        // Exit States >= 20 NONE

        // NULL STATE
        nullState = -1
    };

    template <typename real>
    class ControlManagerV2
    {
    private:
        // Objects
        Controller<real> m_pid;

        // Private Properties

        // FSM Current State
        States m_currentState;
        States m_lastState;

        // Direction Vector Set Point
        real m_angleSP; // Random Value
        Vector2<real> m_vSP;

        // Direction Vector Measured Value
        real m_angleMV; // Compass
        Vector2<real> m_vMV;

        // PID experimental
        // CCW: left versus CW: right (critical)
        bool m_directionCCW;
        // Using sign to determine turn direction.
        real m_perpDot;
        // Dot Product (not critical)
        real m_dot;
        // Dot Angle for uf (critical)
        real m_dotAngle;
        // control function (uf) uses error function (ef)
        real m_uf;

        // Motors Speeds (16-bit word integers only)
        // Use 10-bit digits (0 - 1023)
        //   0 ---- 511 ---- 1023 (I2C data as 16-bit word)
        //  -1 ----   0 ---- 1    (Joystick Algorithm floats)
        // Once data is received by the Slave-MCU use
        // mapping to convert 10-bit digits to floats...
        int m_xSpeedInteger; // used for turn only
        int m_ySpeedInteger; // used for forward only
        // Both may be used together...

        // Private Setters
        void m_setAngleSP(real angle);
        void m_setAngleMV(real angle);

        // Private Methods
        void m_randomAngle();
        void m_angleToVectorSP(); // (critical)
        void m_angleToVectorMV(); // (critical)
        real m_sign(real value);

        // Private States
        bool m_startState();
        bool m_enterSetpointState();
        bool m_setpointState();
        bool m_turnDirectionState();
        bool m_orientationState();

        // Private Supporting Update Methods
        Vector2<real> m_directionVector(real radian);
        bool m_turnChanged();
        bool m_stateChanged();
        void m_turnDirection();
        void m_updatePID(); // ------------------------ Update
        void m_updateFSM(); // ------------------------ Update

        // Debug
        String m_printState();
        void m_debugPlotCSV();

    public:
        // Constructor - inline
        ControlManagerV2()
        {
            // Critical Variables
            m_angleSP = (real)0;
            m_angleMV = (real)0;
            m_dot =  (real)0;
            m_dotAngle = (real)0;

            // Current State ID
            m_currentState = States::StartState;
            m_lastState = States::nullState;

            // PID Controller
            real Kp = (real)0.95;
            real Ki = (real)0.0;
            real Kd = (real)0.0;
            // BEWARE: Sampled-Interval Ts
            // should have the same time
            // interval as loop() timer...
            // MASTER --------------------------------------- WATCHME
            real Ts = (real)0.25; // See main.cpp loop()...
            
            // DEBUG ---------------------------------------- WATCHME
            //real Ts = (real)1; // See main.cpp loop()...

            m_pid = Controller<real>(Kp, Ki, Kd, Ts);
            // Assumming control function (uf)
            // approaches zero...
            m_pid.SetPoint((real)0);
            m_uf = (real)0;
        }

        ~ControlManagerV2() = default;

        /////////////////////////////////////////////////////////////
        // ControlManagerV2 loop() inline
        void Update(real heading, bool debug)
        {
            m_angleMV = heading;

            // Set Measured Value
            m_setAngleMV(m_angleMV);

            // Require both Vector SP and Vector MV
            m_angleToVectorMV();

            // PID
            m_updatePID();

            // FSM where (m_lastState) used in m_stateChanged()
            m_updateFSM();
            // Call m_updateFSM() before assigning m_lastState...
            m_lastState = m_currentState;

            // CSV - Debug Comment this or the other....
            if (debug)
                m_debugPlotCSV();
        }
        /////////////////////////////////////////////////////////////

        // Getters
        real GetAngleSP();
        real GetAngleMV();
        int SpeedIntegerX();
        int SpeedIntegerY();
    };

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
    int ControlManagerV2<real>::SpeedIntegerX()
    {
        return m_xSpeedInteger;
    }

    template <typename real>
    int ControlManagerV2<real>::SpeedIntegerY()
    {
        return m_ySpeedInteger;
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
            turnChanged = m_perpDot >= (real)0;
        else
            turnChanged = m_perpDot < (real)0;

        return turnChanged;
    }

    template <typename real>
    void ControlManagerV2<real>::m_turnDirection()
    {
        if (m_perpDot < 0)
            m_directionCCW = true; // Left-turn
        else
            m_directionCCW = false; // Right-turn
    }

    template <typename real>
    bool ControlManagerV2<real>::m_stateChanged()
    {
        if ((int)m_currentState != (int)m_lastState)
            return true;
        return false;
    }

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
        // Random Angle used as PID Setpoint
        m_randomAngle();

        // Transition
        m_currentState = States::TurnDirectionState;

        // Auto Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_turnDirectionState()
    {
        // Set the turn direction
        m_turnDirection();

        // Transition
        m_currentState = States::OrientationState;

        // Auto Flag
        return true;
    }

    template <typename real>
    bool ControlManagerV2<real>::m_orientationState()
    {
        // There's a lot going on here --------------- FIXME
        // Arbitrary Values
        bool transitionflag = false;
        bool turnflag = m_turnChanged();
        // turn direction will constantly switch
        // back and forth until a m_uf is caught...
        // Units Degrees
        bool spFlag = abs(m_uf) < (real)2.5;
        bool xMotorsFlag = abs(m_uf) > (real)15; // 4x spFlag

        // All sets MUST use if-else ------------------ FIXME
        // Prevents a bug from catching a last (if) statement...
        //
        // Priority 0 - Arbitrary Speeds for now
        if (xMotorsFlag)
        {
            // Right Turn only 
            //  0 ---- 511 --*- 1023 (818)
            // -1 ----   0 --*- 1    (0.8) 80% power
            // Left Turn only 
            //  0 -*-- 511 ---- 1023 (204)
            // -1 -*--   0 ---- 1    (-0.8) 80% power
            if (m_directionCCW)
                m_xSpeedInteger = 767; // CCW
            else
                m_xSpeedInteger = 255; // CW

            // Stop yMotors
            m_ySpeedInteger = 511;
        }
        else
        {
            // Turn slowly
            // Right Turn only 
            //  0 ---- 511 -*-- 1023 (639)
            // -1 ----   0 -*-- 1    (0.625)  62.5% power
            // Left Turn only 
            //  0 --*- 511 ---- 1023 (383)
            // -1 --*-   0 ---- 1    (-0.625)  62.5% power
            if (m_directionCCW)
                m_xSpeedInteger = 639; // CCW
            else
                m_xSpeedInteger = 383; // CW

            // Move forward
            //  0 ---- 511 --*- 1023 (818)
            // -1 ----   0 --*- 1    (0.8) 80% power
            m_ySpeedInteger = 818;
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
            m_currentState = States::TurnDirectionState;
            transitionflag = true;
        }

        // Flagged Transition
        return transitionflag;
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

        case States::TurnDirectionState:
            // Must have a State Method
            if (m_turnDirectionState())
            { /* Allow State to Decide */
            }
            break;

        case States::OrientationState:
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

        case States::TurnDirectionState:
            // str = "Turn Direction State";
            str = "TS"; // Use this for CSV
            break;

        case States::OrientationState:
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
}

#endif
