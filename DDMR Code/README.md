## DDMR code

- namespace **csm** - control state machine...

## ***include folder***

**Following classes are stand-alone**
- **Bitwise.h** 
- **BusI2C.h** 
- **Common.h** 
- **Compass.h** 
- **Controller.h**   
    - file located in PID repository (ok not to move for now)
- **ControlManagerQ.h**   
    - new pending...
- **ControlManagerV2.h**
    - completed
- **Joystick.h**
    - **Archived version** using namespace csm (control state machine). Do not use the other Joystick.h class using namespace uno...
    - This copy remains with DDMR-Orientation repository...
- **L298N.h** 
    - **Archived version** using namespace csm (control state machine). Do not use the other L298N.h class using namespace uno...
    - This copy remains with DDMR-Orientation repository...
- **LinearMap.h** 
- **LSM303.h** 
- **Switch.h** 
    - Instead of using a tactile button, a single wire is used as a switch...
    - TODO - interchange Button.h class to debug... 
- **Timer.h**  
    - file located in Numerics repository (ok not to move for now)
- **Vector2.h** 
    - file located in Numerics repository (ok not to move for now)

## ***src folder***

- Step1STATES folder
    - Quaternions
        - main.cpp
            - **pending...**
    - Vector2
        - main.cpp
            - **Step-1 States**: development completed...
                - (1) Vector2 used for directional unit vectors: one for the setpoint and the other for the measured value. The setpoint vector originated from a random angle and the measured value vector originated from the compass heading (LSM303).
                - (2) Perpendicular-Dot Product on the unit vectors, the vector setpoint and vector measured value) were used for turning-direction (CW or CCW). The sign of the product gave the shortest arc length (or distance), thus giving a turn direction. The product may not be familiar to most CG gurus, so I will post a brief article here...
                - (3) **Angle Dot Product** on the unit vectors (vector Setpoint and vector Measured Value) used as the PID control function (uf).
- Step2MasterI2C folder
    - main.cpp
        - development similar to MCU Communication - Completed...
- Step2SlaveI2C folder
    - main.cpp
        - development similar to MCU Communication yet this has become a study along with the control manager for the direction vectors... Completed...
- XCPP folder
    - **BusI2C.cpp** 
    - **LSM303.cpp**

## ***root folder***

- platformio.ini
    - (Please review this document for platform configuration while testing the code especially for the src folder...)

## Notes


