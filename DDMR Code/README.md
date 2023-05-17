## DDMR code

- namespace - there is no specificity but does incorporate others...

## ***include folder***

**Following classes are stand-alone**
- **BusI2C.h** 
- **Controller.h**   
    - file located in PID repository (ok not to move for now)
- **LSM303.h** 
- **ManagerFSM.h**    
- **Statistics.h** 
    - file located in Numerics repositiry (ok not to move for now)
- **Timer.h**  
    - file located in Numerocs repository (ok not to move for now)
- **Vector2.h** 
    - file located in Numerocs repository (ok not to move for now)

## ***src folder***

- Step1STATES folder
    - main.cpp
        - **work in progress...**
- Step2MOTORS folder
    - main.cpp
        - pending (empty)
- TESTS folder
    - Common.h 
        - empty
- XCPP folder
    - **BusI2C.cpp** 
    - **LSM303.cpp**

## ***root folder***

- platformio.ini
    - (Please review this document for platform configuration while testing the code.)

## Notes

- A few header files were split into their implementation files with the extension (.cpp)... This may change...

