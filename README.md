# dc-motor-pid-control
Closed-loop PID velocity and position control of a DC gearmotor with quadrature encoder feedback, implemented in C++ on Arduino

### Overview
This project implements a full PID control loop for a DC gearmotor, using quadrature encoder feedback to achieve precise velocity and position control. The controller is written from scratch in C++ to demonstrate direct 
understanding of proportional, integral, and derivative control behavior. The goal is to characterize how each PID term affects system response, tune gains systematically, and validate performance against defined specifications using real-time serial data logging and Python-based analysis. As of now, only positional control is implemented.

### Hardware
- Microcontroller:   Arduino Uno
- Motor Driver:      H-Bridge (L298N)
- Motor:	           DC Gearmotor with quadrature encoder
- Encoder:	         Quadrature (A/B channels), interrupt-driven
- Power Supply:	     External 12V supply for motor, USB for Arduino

### Wiring overview
Encoder A and B channels connected to Arduino interrupt pins (D2, D3)
H-bridge IN1/IN2 for direction control, ENA for PWM speed control
Serial output at 9600 baud for real-time data logging

### File Structure
- **arduino:** contains motor_pid.ino. This is the c++ code that gets uploaded to the arduino via usb. Individual test cases can be run from this file using the arduino IDE and viewing the output in a serial plotter by uncommenting "Alternate testing printing" and commenting out "print outputs to serial". 
- **analysis:** contains serial_logging.py, the main file to run all test cases. Test cases are configured in the Test Matrix section, choosing step sizes, directions, startin condition, and number of trials.  
- **pid_test_logs:** contains test cases. Each file contains 
  - summary.csv: a summary of every test case run
  - params.txt: text file describing the pid parameters used
  - 3 plots: steady state error, settling time, and % overshoot over 2 degrees

### Testing Specification
A test is passed if |current_angle - target| <= TOLERANCE_DEG (currently 3.0°) at that sample, and
it stays within that band for every sample afterward through the end of capture. In addition, we capture the following metrics for each test
- Steady state error: the absolute 


Position control accuracy:	±2 degrees steady-state error
Settling time:	< 500ms for a 90-degree step input
Velocity control:	Stable tracking within ±5 RPM of setpoint
Overshoot:	< 10%

### Encoder Reading

### PID Control Loop

### Data Logging

### Results

### Lessons Learned
