# Picasso_tMotor_Control_Python_and_Arduino

This repo contains microcontroller control code for controlling tMotor QDD actuators. This microcontroller code is written for Arduino compatible boards, specifically the Teensy 4.1. We will be working on supporting more microcontrollers in future.

The repo also has a python header program that can be used to do some basic torque tracking to get you started. This code requires a serial (UART) connection between your microcontroller and the pc running the python program.

The Arduino_C++ Drivers folder contains the driver files (written in c++ with a header file for conversion to arduino) for basic motor control. 

Note that your tMotor actuators will need to be set to "MIT" mode using tMotor's motor setup software before this code will work.
