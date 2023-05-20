//
// Carpenter Software
// File: Class ManagerFSM.h
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

#ifndef Manager_FSM_h
#define Manager_FSM_h

#include <Arduino.h>
#include <Wire.h>

#include "BusI2C.h"
#include "LSM303.h"
#include "Vector2.h"
#include "Controller.h"
#include "Statistics.h"
#include "Timer.h"

using namespace dsg;
using namespace nmr;
using namespace pid;

// MASTER UNO BOARD
#define MASTER_ADDR_0x14 0x14

// SLAVE Atmega328P Breadboard
// Used for motors....
// #define SLAVE_ADDR_0x16 0x16

// TUPLES_SIZE 6
#define TUPLES_SIZE 6

namespace fsm
{
    enum States
    {
        // States < 10
        // Reverse Order
        IdleState = 9,
        SetpointState = 8,
        TurnDirectionState = 7,
        OrientationState = 6,

        // Enter States >= 10
        // Reverse Order
        EnterIdleState = 19,
        EnterSetpointState = 18,
        EnterTurnDirectionState = 17,
        EnterOrientationState = 16

        // Exit States >= 20 NONE
    };

    template <typename real>
    class ManagerFSM
    {
    private:
        // Objects
        BusI2C m_busI2C;
        LSM303 m_compass;
        Controller<float> m_pid;
        Statistics<float> m_stats;
        Timer m_debugTimer;

        // Private Properties
        States m_currentState;
        real m_angleSP;
        real m_angleMV;

        // experimental
        // CCW: left
        bool m_directionCCW;
        real m_perpDot;
        real m_dot;
        real m_uf;

        // Processing Orientation
        bool m_IsProcessingOrientation;

        // Statistics
        real m_tuples[TUPLES_SIZE];
        int m_index;
        real m_mean;
        real m_sd;

        // Direction Vector Set Point
        Vector2<real> m_vSP;

        // Direction Vector Measured Value
        // Value = Magnetometer....
        Vector2<real> m_vMV;

        // Private Setters
        void m_setAngleSP(float angle);
        void m_setAngleMV(float angle);

        // Private Quires
        void m_processingOrientation();
        void m_releaseOrientation();
        bool m_isProcessingOrientation();

        // Private Methods
        void m_randomAngle();
        void m_angleToVectorSP();
        void m_angleToVectorMV();

        // Private States
        bool m_enterIdleState();
        bool m_idleState();
        bool m_enterSetpointState();
        bool m_setpointState();
        bool m_enterOrientationState();
        bool m_orientationState();
        bool m_enterTurnDirectionState();
        bool m_turnDirectionState();

        // Private Supporting Update Methods
        Vector2<real> m_directionVector(real radian);
        void m_updateCompass();
        void m_updatePID();
        bool m_turnChanged();
        real m_statistics(float value);
        void m_updateFSM();

        // Debug
        void m_debugVitals();
        String m_printState();

    public:
        // Constructor - inline
        ManagerFSM()
        {
            m_angleSP = (real)0;
            m_angleMV = (real)0;
            m_IsProcessingOrientation = false;

            // I2C Bus
            m_busI2C = BusI2C();

            // Compass
            m_compass = LSM303();

            // Current State ID
            m_currentState = States::EnterIdleState;

            // PID Controller
            real Kp = (real)0.95;
            real Ki = (real)0.0;
            real Kd = (real)0.0;
            // BEWARE: Sampled-Interval Ts
            // has to have the same time
            // interval as loop() timer...
            real Ts = (real)1.0;
            m_pid = Controller<float>(Kp, Ki, Kd, Ts);
            // Assumming error approaches zero...
            m_pid.SetPoint((real)0);

            // Statistics (optional)
            // Tuples used to get Mean & SD...
            // Used inside m_updateCompass()
            for (int i = 0; i < TUPLES_SIZE; i++)
                m_tuples[i] = (real)0;
            m_stats = nmr::Statistics<float>(m_tuples, TUPLES_SIZE);
            // Index counter for Queue()
            m_index = 0;

            // Timer
            m_debugTimer = Timer();
        }

        ~ManagerFSM() = default;

        // ManagerFSM setup()
        void Begin();
        // ManagerFSM loop() inline
        void Update()
        {
            // Heading
            m_updateCompass();
            m_updatePID();
            m_updateFSM();

            // loop() timer is fast
            if (m_debugTimer.isTimer(1000))
                m_debugVitals();
        }

        // Getters
        float GetAngleSP();
        float GetAngleMV();
    };

    template <typename real>
    void ManagerFSM<real>::Begin()
    {
        Serial.println("Begin");

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
    float ManagerFSM<real>::GetAngleSP()
    {
        return m_angleSP;
    }

    template <typename real>
    float ManagerFSM<real>::GetAngleMV()
    {
        return m_angleMV;
    }

    // PRIVATE SETTERS

    template <typename real>
    void ManagerFSM<real>::m_setAngleSP(float angle)
    {
        m_angleSP = angle;
    }

    template <typename real>
    void ManagerFSM<real>::m_setAngleMV(float angle)
    {
        m_angleMV = angle;
    }

    template <typename real>
    void ManagerFSM<real>::m_processingOrientation()
    {
        m_IsProcessingOrientation = true;
    }

    template <typename real>
    void ManagerFSM<real>::m_releaseOrientation()
    {
        m_IsProcessingOrientation = false;
    }

    template <typename real>
    bool ManagerFSM<real>::m_isProcessingOrientation()
    {
        return m_IsProcessingOrientation;
    }

    // PRIVATE METHODS

    template <typename real>
    void ManagerFSM<real>::m_randomAngle()
    {
        real angle;
        real last = GetAngleSP();
        bool flag = true;
        while (flag)
        {
            // Creates Random Angle from 0 to 360
            angle = (real)(rand() % 360);

            // The difference is equal or greater
            // than 30 degrees...
            if (abs(angle - last) >= 30)
            {
                // float
                m_setAngleSP(angle);
                flag = false;
            }
        };

        // Convert SP angle to vector
        m_angleToVectorSP();
    }

    template <typename real>
    void ManagerFSM<real>::m_angleToVectorSP()
    {
        float rad = GetAngleSP() * DEG_TO_RAD;
        m_vSP = m_directionVector(rad);
    }

    template <typename real>
    void ManagerFSM<real>::m_angleToVectorMV()
    {
        float rad = GetAngleMV() * (float)DEG_TO_RAD;
        m_vMV = m_directionVector(rad);
    }

    // PRIVATE METHODS

    template <typename real>
    Vector2<real> ManagerFSM<real>::m_directionVector(real radian)
    {
        // MiscMath to Vector2
        float a = cos(radian);
        float b = sin(radian);
        Vector2<float> vector(a, b);
        return vector;
    }

    template <typename real>
    void ManagerFSM<real>::m_updateCompass()
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

        // Use statistcal average (5-point data queue)
        // OPTIONAL (can be commented out)
        angle = m_statistics(angle);

        // Set Measured Value
        m_setAngleMV(angle);

        // Require both Vector SP and Vector MV
        m_angleToVectorMV();

        // Prevent hiccups (stalls)
        m_busI2C.ClearTimeout();
    }

    template <typename real>
    void ManagerFSM<real>::m_updatePID()
    {
        // Determine Turning Direction
        m_perpDot = m_vMV.PerpDot(m_vSP);
        m_dot = m_vMV.Dot(m_vSP);
    }

    template <typename real>
    bool ManagerFSM<real>::m_turnChanged()
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
    real ManagerFSM<real>::m_statistics(float value)
    {
        // Statistics (5-point data queue)
        // See ManagerFSM CLASS
        if (m_index >= TUPLES_SIZE)
            m_index = 0;
        m_stats.Queue(value, m_index);
        m_mean = m_stats.Average();
        m_sd = m_stats.StandardDeviation();
        m_index++;
        return m_mean;
    }

    // STATES

    template <typename real>
    bool ManagerFSM<real>::m_enterIdleState()
    {
        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_idleState()
    {
        // Nothing to do here...

        // Prepare FSM to Transition To Setpoint State
        m_currentState = States::EnterSetpointState;

        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_enterSetpointState()
    {
        // An Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_setpointState()
    {
        // Choose a random angle (-180 tp 180) as the set-point (SP)
        // and greater than the previous angle by 30 degrees.
        m_randomAngle();

        // Prepare FSM to Transition To Turn Direction State
        m_currentState = States::EnterTurnDirectionState;

        // Ready Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_enterTurnDirectionState()
    {
        // Ready Auto-Transitional Flag
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_turnDirectionState()
    {
        if (m_perpDot < 0)
            m_directionCCW = true; // Left-turn
        else
            m_directionCCW = false; // Right-turn

        m_currentState = States::EnterOrientationState;
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_enterOrientationState()
    {
        m_processingOrientation();
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_orientationState()
    {
        bool flag = false;

        // Priority
        if (m_turnChanged())
        {
            m_currentState = States::EnterTurnDirectionState;
            return true;
        }

        // PID Control
        if (m_isProcessingOrientation())
        {
            // PID Setpoint
            // Initially set within the constructor (zero)
            // PID Process Point
            real processPoint = ((real)1.0 - m_dot) * (real)189;
            // Kp = 0.95, Ki = 0, Kd = 0 (see constructor)
            // The '189' is an Emperical Control Factor used to 
            // match the magnitude to the degrees 0 to 360. This
            // may have to change based on the Kp factor.
            // The dot product is linear....
            m_pid.UpdatePID(processPoint);
            // PID Control Function
            m_uf = m_pid.Control();
        }

        // Priority
        // MOTORS - '10' arbitrary value which
        // approximately 10 degrees....
        if (abs(m_uf) > (real)10)
        {
            // Using Joystick Algorithm
            // Turn Only (x)
            // where x is a constant if
            // y remains zero...
        }
        else
        {
            // Using Joystick Algorithm
            // Turn (x)
            // Forward (y)
            // where y will change x differentially
            // from 0 to 1... Control???
        }

        // Time Constraint -
        if (flag)
            m_currentState = States::EnterIdleState;

        // Flagged Transition
        return flag;
    }

    template <typename real>
    void ManagerFSM<real>::m_updateFSM()
    {
        // Finite State Machine
        switch (m_currentState)
        {
        case States::EnterIdleState:
            // Must have a State Method
            if (m_enterIdleState())
            {
                // Transition To...
                m_currentState = States::IdleState;
            }
            break;

        case States::IdleState:
            // Must have a State Method
            if (m_idleState())
            { /* Allow State to Decide */
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

        default:
            break;
        }
    }

    template <typename real>
    void ManagerFSM<real>::m_debugVitals()
    {
        // Current State
        Serial.println(m_printState());

        // HEADING Statistics
        Serial.print("Heading avg: ");
        Serial.print(m_mean);
        Serial.print(" sd: ");
        Serial.println(m_sd);

        // Setpoint (vector)
        Serial.print(GetAngleSP());
        Serial.print(" vSP: (");
        Serial.print(m_vSP.x());
        Serial.print(", ");
        Serial.print(m_vSP.y());
        Serial.println(")");

        // Measrured Value (vector)
        Serial.print(GetAngleMV());
        Serial.print(" vMV: (");
        Serial.print(m_vMV.x());
        Serial.print(", ");
        Serial.print(m_vMV.y());
        Serial.println(")");

        // MOTORS
        // Joystick Algorithm
        if (abs(m_uf) > (real) 10)
            Serial.print("T: "); // Turn Only (x)
        else
            Serial.print("TF: "); // Turn and Forward (x, y)

        // PID - Looking for the shortest
        // circumference (distance) while
        // using a calibrated compass approaching
        // the Setpoint (desired) value.... 
        if (m_perpDot > 0)
            Serial.print(" Right perpDot: ");
        else
            Serial.print(" Left perpDot:");

        // Once the Left and Right are established,
        // Make a note which perpDot sign (-/+) is
        // used for which Left and Right directions...
        // (-) is left
        // (+) is right
        // See m_turnDirectionState() for details...
        Serial.print(m_perpDot);
        Serial.print(" dot: ");
        Serial.print(m_dot);
        Serial.print(" uf: ");
        Serial.println(m_uf);

        Serial.println("---------------------------");
    }

    template <typename real>
    String ManagerFSM<real>::m_printState()
    {
        String str = "";

        // Shoud match updateFSM()
        switch (m_currentState)
        {
        case States::EnterIdleState:
            str = "Enter Idle State";
            break;

        case States::IdleState:
            str = "Idle State";
            break;

        case States::EnterSetpointState:
            str = "Enter Setpoint State";
            break;

        case States::SetpointState:
            str = "Setpoint State";
            break;

        case States::EnterTurnDirectionState:
            str = "Enter Turn Direction State";
            break;

        case States::TurnDirectionState:
            str = "Turn Direction State";
            break;

        case States::EnterOrientationState:
            str = "Enter Orientation State";
            break;

        case States::OrientationState:
            str = "Orientation State";
            break;
        }
        return str;
    }
}

#endif