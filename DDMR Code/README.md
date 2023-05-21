## DDMR code

- namespace - there is no specificity but does incorporate others...

## ***include folder***

**Following classes are stand-alone**
- **BusI2C.h** 
- **Controller.h**   
    - file located in PID repository (ok not to move for now)
- **LSM303.h** 
- **ControlManagerQ.h**   
    - new pending...
- **ControlManagerV2.h**
    - name change from ManagerFSM.h
- **Statistics.h** 
    - file located in Numerics repositiry (ok not to move for now)
- **Timer.h**  
    - file located in Numerocs repository (ok not to move for now)
- **Vector2.h** 
    - file located in Numerocs repository (ok not to move for now)

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
                - (3) Dot Product on the unit vectors (vector Setpoint and vector Measured Value) used as the PID error function (ef).
- Step2MOTORS folder
    - main.cpp
        - **pending...**
- TESTS folder
    - Common.h 
        - **pending...**
- XCPP folder
    - **BusI2C.cpp** 
    - **LSM303.cpp**

## ***root folder***

- platformio.ini
    - (Please review this document for platform configuration while testing the code.)

## Notes

- A few header files were split into their implementation files with the extension (.cpp)... This may change...

