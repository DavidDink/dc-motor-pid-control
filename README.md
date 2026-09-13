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
    
### PID Control Loop
5 main parameters were used to tune the PID control loop. The PID loop runs on a fixed clock defined by pidInterval in motor_pid.ino so that any serial logging or print statements don't interfere with the pid loop time calculations.
- **tolerance = 2.8:** A deadband, friction constant only applies outside the deadband.
- **friction = 30:** a constant tuned to overcome static friction for a 45:1 gear reduction used to drive the motor in the direction of error
- **Kp = 2.0**: Proportional term
- **Ki = 0.3**: Integral term. The integral gets reset every time a new command is typed in.
- **Kd = 0.4**: Derivative term
- PID OUTPUT = Kp * error + Ki * integral + Kd * derivative + friction

### Testing Specification
A test is passed if |current_angle - target| <= TOLERANCE_DEG (currently 3.0°) at that sample, and
it stays within that band for every sample afterward through the end of capture. In addition, we capture the following metrics for each test
- Steady state error: the mean absolute error over the last STEADY_STATE_WINDOW_SEC (currently 1.0s) of the captured response
- Asymptotic settling time: the time elapsed to reach TOLERANCE_DEG minus the time it would take to reach TOLERANCE_DEG if motor spun at max speed (130 rpm)
- Overshoot rate: the % of tests that overshoot the target by > 2 degrees.
  
150 test cases were ran for the final test, the test matrix was defined by combinations of the following parameters
- STEP_SIZES_DEG: [10,45,90,180,360] 5 different step sizes used
- DIRECTIONS: [-1,1] Each step size conducted forwards and backwards
- STARTING_CONDITIONS: ["rest","in_motion_same","in_motion_reversal"] for each test, run from rest, run from moving motor in same direction for LAY_SEC seconds (currently 0.3) before switching to target, and run from moving motor in the opposite direction for LAY_SEC before moving in the opposite direction
- REPEATS = 5 number of trials to run for each test

### Results
- pass rate: 100%
- mean asymptotic settling time: 0.4s
- mean steady state error: 1.771 degrees
- overshoot rate over 2 degrees: 0%
- details for each step size shown graphically in pid_test_logs

### Lessons Learned
- **Understand physical limitations before definining testing requirements:** Substantial time went into gain tuning against what turned out to be static friction. Understanding physical limits of the system was critical too. With 1080 ticks per 360 degree revolution, anything movement under 0.3 degrees is less than 1 encoder count and has no way of being recorded. The static friction threshold was found to be 2 degrees (includes gear slop), meaning a movement of 2 degrees or less resulted in no motor shaft movement for any amount of time. This led to defining a passing test as 3 degrees of steady state error, and it also led to creating a constant friction term active outside a deadband window, as accuracy was bounded more by sticton and gearbox effects rather than tuning.
- **Instrumentation requires as much (or more) debugging than PID behavior**: There were measurement artifacts to handle while serial logging like stale serial lines landing as zero and corrupting the relative-time baseline, and capture windows that were too short for large steps resulting in remaining travel distance reporting as steady state error. The fix here was to not infer which serial lines belong to which test based on timing. I added a tag command with an ID for each serial line so that each line carried its own identity, and the logger could filter strictly on ID rather than guessing from arrival time.
- **Track PID errors as they build up over time**: Cranking Ki can be tempting to result in faster convergence, but Ki can linger and affect later target approaches. The Ki term results in a high inertia PID output that doesn't always reflect the output needed to close the gap on the latest target. I found it best to just reset Ki after each target commmand
- **Verify vendor specs, don't assume them**: The datasheet on the JGA25-371 DC gearmotor with encoder was 540 PPR, but described single edge counting. I chose to count both edges for finer degree measurement detail so that the real callibration constant was 1080, confirmed by hand rotating the shaft as well. 
  
