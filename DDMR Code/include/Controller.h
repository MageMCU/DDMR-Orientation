#ifndef PID_Controller_h
#define PID_Controller_h

#include "Arduino.h"

namespace pid
{
    // The Controller Class requires a timing interval at a constant rate...
    // Use the Timer.h object in the loop() function...
    // Note: ControlManager under development...under development
    template <typename real>
    class Controller
    {
    public:
        // Constructor
        Controller() = default;
        Controller(real Kp, real Ki, real Kd, real Ts);
        ~Controller() = default;

        // Getters
        real Error();
        real Integral();
        real Derivative();
        real Control();

        // Setters
        void SetPoint(real setPoint);

        // Methods
        real UpdatePID(real processPoint);

        // Strings
        String PrintError();
        String PrintIntegral();
        String PrintDerivative();
        String PrintEID();
        String PrintFpFiFd();

    private:
        // Inputs
        real m_setPoint;
        // Errors
        real m_last_ef;
        real m_last2_ef;
        // Control
        real m_uf;
        real m_last_uf;
        // Proportional
        real m_proportional;
        // Integral
        real m_integral;
        // Derivative
        real m_derivative;

        // Controller Gains
        real m_Kp;
        real m_Ki;
        real m_Kd;

        // Sampled Time (deltaTime) constant
        real m_Ts;

        // Private Methods

    };

    // Constructor

    template<typename real>
    Controller<real>::Controller(real Kp, real Ki, real Kd, real Ts)
    {
        // Error Function
        m_last_ef = (real)0;
        // Control Function
        m_uf = (real)0;
        m_last_uf = (real)0;
        // integral
        m_integral = (real)0;
        // Controller Gains
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
        // Sampled Time (deltaTime) constant
        m_Ts = Ts;
    }

    // Getters

    template<typename real>
    real Controller<real>::Error()
    {
        return m_last_ef;
    }

    template<typename real>
    real Controller<real>::Integral()
    {
        return m_integral;
    }

    template<typename real>
    real Controller<real>::Derivative()
    {
        return m_derivative;
    }

    template<typename real>
    real Controller<real>::Control()
    {
        return m_uf;
    }

    // Setters

    template<typename real>
    void Controller<real>::SetPoint(real setPoint)
    {
        m_setPoint = setPoint;
    }

    // Methods

    template<typename real>
    real Controller<real>::UpdatePID(real measuredValue)
    {
        // Finite PID Approximations - sampled-Interval Ts
        // (1) (Van de Vegte, FCS, 1986, p232) 
        // (2) (Digital Control 978-1-349-21550-8)
        // (3) Added PID considerations not used here...
        // Basic Elements (Keep it simple for testing)

        // Error Function
        real ef = m_setPoint - measuredValue;

        // Integral Finite-Difference Approximation
        // Integral (backward rectangular rule)
        real integral = m_integral + ef * m_Ts;

        // Alternate Integral Approximation:
        // real eI = ef + (2 * m_last_ef) + m_last2_ef;
        // real integral = m_integral + (0.5 * eI * m_Ts);
        // Source: 
        // 7. Discrete PID Control
        // Dr James E. Pickering
        // https://www.youtube.com/watch?v=LcsaEF-i1j
        // NOT USED

        // Clamp integrator anti-windup output (Hi & Lo)
        // NOT USED

        // Derivative Finite-Difference Approximation
        // Derivative (backward difference)
        real derivative = (ef - m_last_ef) / m_Ts;

        // Derivative Filter: NOT USED

        // Besides the backward, there are forward
        // and central approximations...

        // Store Values for csv-file
        m_last_ef = ef;
        m_integral = integral;
        m_derivative = derivative;

        // Control  Function
        real uf = (m_Kp * ef) + (m_Ki * integral) + (m_Kd * derivative);

        // U-Velocity Algorithm: uV = uf - m_last_uf;
        // m_last_uf = m_uf;
        m_uf = uf;
        // return uf - m_last_uf; // ------------------------- uV

        // U-Positional Algorithm: uP = uf
        return uf; // ---------------------------------------- uP

        // Zero Order Hold (ZOH) subroutine u*f: NOT USED
    }

    // Strings

    template<typename real>
    String Controller<real>::PrintError()
    {
        String str = String(m_last_ef, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintIntegral()
    {
        String str = String(m_integral, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintDerivative()
    {
        String str = String(m_derivative, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintEID()
    {
        // Prints functions as csv data for plotting...
        String str = String(m_last_ef, 2) + "," +
            String(m_integral, 2) + "," +
            String(m_derivative, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintFpFiFd()
    {
        real Fp = m_Kp * m_last_ef;
        real Fi = m_Ki * m_integral;
        real Fd = m_Kd * m_derivative;

        // Prints functions as csv data for plotting...
        String str = String(Fp, 2) + "," +
            String(Fi, 2) + "," +
            String(Fd, 2);
        return str;
    }

    // Private Methods
}

#endif
