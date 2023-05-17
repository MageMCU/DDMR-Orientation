

#ifndef Manager_FSM_h
#define Manager_FSM_h

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

namespace fsm
{
    enum States
    {
        EnterIdleState,
        IdleState,
        EnterSetpointState,
        SetpointState,
        EnterOrientationState,
        OrientationState
    };

    template <typename real>
    class ManagerFSM
    {
    private:
        // Objects
        BusI2C m_busI2C;
        LSM303 m_compass;
        States m_states;
        Controller<float> m_pid;
        Statistics<float> m_stats;

        // Private Properties
        int m_stateID;
        real m_errorPID;
        real m_angleSP;
        real m_angleMV;

        // Processing Orientation
        bool m_IsProcessingOrientation;

        // Statistics
        real m_tuples[5];
        int m_index;
        real m_mean;
        real m_sd;

        // Direction Vector Set Point
        Vector2<real> m_vSP;

        // Direction Vector Measured Value
        // Value = Magnetometer....
        Vector2<real> m_vMV;

        // Private Setters
        void m_setStateID(int stateID);
        void m_setAngleSP(float angle);
        void m_setAngleMV(float angle);
        void m_setErrorPID(float error);

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

        // Private Methods
        Vector2<real> m_directionVector(real radian);
        void m_updateCompass();
        void m_updatePID();
        real m_statistics(float value);
        void m_updateFSM();

    public:
        // Constructor - inline
        ManagerFSM()
        {
            m_stateID = 0;
            m_angleSP = (real)0;
            m_angleMV = (real)0;
            m_errorPID = (real)0;
            m_IsProcessingOrientation = false;

            // I2C Bus
            m_busI2C = BusI2C();

            // Compass
            m_compass = LSM303();

            // Current State ID
            m_states = States::EnterIdleState;

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
            for (int i = 0; i < 5; i++)
                m_tuples[i] = (real)0;
            m_stats = nmr::Statistics<float>(m_tuples, 5);
            // Index counter for Queue()
            m_index = 0;
        }

        ~ManagerFSM() = default;

        // ManagerFSM setup()
        void Begin();
        // ManagerFSM loop() inline
        void Update()
        {
            m_updateCompass();
            m_updatePID();
            m_updateFSM();
        }

        // Getters
        int GetStateID();
        float GetAngleSP();
        float GetAngleMV();
        float GetErrorPID();
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
    int ManagerFSM<real>::GetStateID()
    {
        return m_stateID;
    }

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

    template <typename real>
    float ManagerFSM<real>::GetErrorPID()
    {
        return m_errorPID;
    }

    // PRIVATE SETTERS

    template <typename real>
    void ManagerFSM<real>::m_setStateID(int stateID)
    {
        m_stateID = stateID;
    }

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
        m_IsProcessingOrientation = false;
    }

    template <typename real>
    void ManagerFSM<real>::m_releaseOrientation()
    {
        m_IsProcessingOrientation = true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_isProcessingOrientation()
    {
        return m_IsProcessingOrientation;
    }

    template <typename real>
    void ManagerFSM<real>::m_setErrorPID(float error)
    {
        m_errorPID = error;
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
            angle = (real)(rand() % 360);
            // Clockwise(+) Counter-Clockwise(-) ------------------- NEW
            if (angle > (real)180)
                angle = angle - (real)360;

            if (abs(angle - last) >= 30)
            {
                // float
                m_setAngleSP(angle);
                flag = false;
            }
        };

        // DEBUG
        // Serial.print("random angle: ");
        // Serial.print(angle);
    }

    template <typename real>
    void ManagerFSM<real>::m_angleToVectorSP()
    {
        float rad = GetAngleSP() * DEG_TO_RAD;
        m_vSP = m_directionVector(rad);

        // DEBUG
        // Serial.print(" vSP: (");
        // Serial.print(m_vSP.x());
        // Serial.print(", ");
        // Serial.print(m_vSP.y());
        // Serial.println(")");
    }

    template <typename real>
    void ManagerFSM<real>::m_angleToVectorMV()
    {

        // Debug
        // Serial.println(GetAngleMV());

        float rad = GetAngleMV() * (float)DEG_TO_RAD;
        m_vMV = m_directionVector(rad);

        // DEBUG
        // Serial.print(" vMV: (");
        // Serial.print(m_vMV.x());
        // Serial.print(", ");
        // Serial.print(m_vMV.y());
        // Serial.println(")");
    }

    // STATES

    template <typename real>
    bool ManagerFSM<real>::m_enterIdleState()
    {
        m_setStateID(0);
        // Serial.print("Idle ");
        // Serial.println(GetStateID());

        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_idleState()
    {
        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_enterSetpointState()
    {
        m_setStateID(1);
        // Serial.print("Setpoint ");
        // Serial.println(GetStateID());

        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_setpointState()
    {
        // Choose a random angle (-180 tp 180) as the set-point (SP)
        // and greater than the previous angle by 30 degrees.
        m_randomAngle();
        // Convert SP angle to vector
        m_angleToVectorSP();

        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_enterOrientationState()
    {
        m_setStateID(2);
        // Serial.print("Orientation ");
        // Serial.println(GetStateID());

        // Set Orientation Boolean
        m_processingOrientation();
        // ReleaseOrientation();

        return true;
    }

    template <typename real>
    bool ManagerFSM<real>::m_orientationState()
    {
        bool flag = m_isProcessingOrientation();

        // Serial.print("Measured Angle: ");
        // Serial.print(GetAngleMV());
        m_angleToVectorMV();

        return flag;
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
        // Compass Update
        m_compass.read();
        // Compass Heading
        real angle = m_compass.heading(); // ------------ FIXME(see NEW)
        // Declination
        real declinationAngle = (8.0 + (3.0 / 60.0));
        angle += declinationAngle;
        // Keep angle between 0 - 360
        if (angle < (real)0)
            angle = angle + (real)360;
        if (angle > (real)360)
            angle = angle - (real)360;
        // Clockwise(+) Counter-Clockwise(-) ------------------- NEW
        if (angle > (real)180)
            angle = angle - (real)360;

        // Use statistcal average (5-point data queue)
        // OPTIONAL (can be commented out)
        angle = m_statistics(angle);

        // Set Measured Value
        m_setAngleMV(angle);

        // Debug
        // Serial.print("Measured Value: ");
        // Serial.print(GetAngleMV());

        // Prevent hiccups (stalls)
        m_busI2C.ClearTimeout();
    }

    template <typename real>
    void ManagerFSM<real>::m_updatePID()
    {
        Serial.println("Update PID");

        // Determine Turning Direction
        real perpDot = m_vMV.PerpDot(m_vSP);

        // Debug PerpDot
        if(perpDot > 0)
            Serial.print("Left: "); // ??????????????????
        else
            Serial.print("Right:");
        Serial.print(perpDot);

        // Dot Product
        real dot = m_vMV * m_vSP;

        // Debug Dot Product
        Serial.print(" Dot: ");
        Serial.println(dot);
    }

    template <typename real>
    real ManagerFSM<real>::m_statistics(float value)
    {
        // Statistics (5-point data queue)
        // See ManagerFSM CLASS
        if (m_index >= 5)
            m_index = 0;
        m_stats.Queue(value, m_index);
        m_mean = m_stats.Average();
        m_sd = m_stats.StandardDeviation();
        m_index++;
        // Debug Statistics
        // Serial.print("Heading avg: ");
        // Serial.print(m_mean);
        // Serial.print(" sd: ");
        // Serial.println(m_sd);

        return m_mean;
    }

    template <typename real>
    void ManagerFSM<real>::m_updateFSM()
    {
        // Finite State Machine
        switch (m_states)
        {
        case States::EnterIdleState:
            if (m_enterIdleState())
                m_states = States::IdleState;
            break;

        case States::IdleState:
            if (m_idleState())
                m_states = States::EnterSetpointState;
            break;

        case States::EnterSetpointState:
            if (m_enterSetpointState())
                m_states = States::SetpointState;
            break;

        case States::SetpointState:
            if (m_setpointState())
                m_states = States::EnterOrientationState;
            break;

        case States::EnterOrientationState:
            if (m_enterOrientationState())
                m_states = States::OrientationState;
            break;

        case States::OrientationState:
            if (m_orientationState())
                m_states = States::EnterIdleState;
            break;

        default:
            break;
        }
    }
}

#endif