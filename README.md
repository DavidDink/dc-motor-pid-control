# dc-motor-pid-control
Closed-loop PID velocity and position control of a DC gearmotor with quadrature encoder feedback, implemented in C++ on Arduino

### Overview
This project implements a full PID control loop for a DC gearmotor, using quadrature encoder feedback to achieve precise velocity and position control. The controller is written from scratch in C++ to demonstrate direct 
understanding of proportional, integral, and derivative control behavior. The goal is to characterize how each PID term affects system response, tune gains systematically, and validate performance against defined specifications using real-time serial data logging and Python-based analysis. As of now, only positional control is implemented.

### Hardware
- Microcontroller:   Arduino Uno
- Motor Driver:      H-Bridge (L298N)
- Motor:	           DC Gearmotor with quadrature encoder. Model: JGA25-371. 130 rpm. 
- Encoder:	         Quadrature (A/B channels), interrupt-driven. 1080 ticks per revolution (counts leading and falling edge)
- Power Supply:	     External 12V supply for motor, USB for Arduino

### Wiring overview
**Arduino**: 
- USB-B to computer for serial. Port configured in serial_logging.py as SERIAL_PORT. Baud rate 9600
- 5v Power to Encoder (Blue wire)
- Ground to shared ground bus
- pin 2 to quad encoder A signal (interupt capable) (yellow wire)
- pin 3 to quad encoder B signal (interupt capable) (white wire)
- pin 7 to h-bridge IN2
- pin 8 to h-bridge IN1
- pin 9 to h-bridge ENA (interupt capable)
  
**Encoder**:
- Red - Motor power terminal (+) to h-bridge out 1
- Black - Motor power terminal(-) to h-bridge out 2
- Green - Quad encoder Ground to shared ground bus
- Blue - Quad encoder +5V Vcc to 5v power on arduino
- Yellow - Quad encoder A signal to arduino pin 2
- White - Quad encoder B signal to arduino pin 3

**H-bridge**:
- Out 1 to motor power terminal (+) (red wire)
- Out 2 to motor power terminal (-) (black wire)
- GND to shared ground bus
- +12V to 12V power supply
- ENA to arduino pin 9
- IN1 to arduino pin 8
- IN2 to arduino pin 7

**12V Power Supply**:
- Positive to +12V on h-bridge
- Negative to shared ground bus

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
