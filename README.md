# dc-motor-pid-control
Closed-loop PID velocity and position control of a DC gearmotor with quadrature encoder feedback, implemented in C++ on Arduino

**Overview**
This project implements a full PID control loop for a DC gearmotor, using quadrature encoder feedback to achieve precise velocity and position control. The controller is written from scratch in C++ — no PID libraries — to demonstrate direct 
understanding of proportional, integral, and derivative control behavior. The goal is to characterize how each PID term affects system response, tune gains systematically, and validate performance against defined specifications using 
real-time serial data logging and Python-based analysis.

**Hardware**
Microcontroller:   Arduino Uno
Motor Driver:      H-Bridge (L298N)
Motor:	           DC Gearmotor with quadrature encoder
Encoder:	         Quadrature (A/B channels), interrupt-driven
Power Supply:	     External 12V supply for motor, USB for Arduino

**Wiring overview:**
Encoder A and B channels connected to Arduino interrupt pins (D2, D3)
H-bridge IN1/IN2 for direction control, ENA for PWM speed control
Serial output at 115200 baud for real-time data logging

**Encoder Reading**

**PID Control Loop**

**Data Logging**

**Results**

**Lessons Learned**
