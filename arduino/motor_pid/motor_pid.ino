unsigned long lastPIDTime = 0;
const unsigned long pidInterval = 10; // fixed PID update rate
long commandID = 0; // a fixed ID per instruction, so that serial logging has a way to identify which action was instructed

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 40; // milliseconds between prints

volatile long pulseCount = 0; // Positive pulse count means CCW rotation

// define constants based on wiring. Default is CCW
const int IN1 = 8; // determine direction. IN1 = HIGH, IN2 = LOW means CCW
const int IN2 = 7; // determine direction. IN1 = LOW, IN2 = HIGH means CW
const int ENA = 9; // 9 is interupt-capable on arudino, will use for PWM to control speed

// pins 2 and 3 are interupt capable (they can interupt the loop and read multiple times instead of waiting for another loop)
const int ENCA = 2; // YELLOW encoder Signal A. 
const int ENCB = 3; // WHITE encoder Signal B. 
const float countsPerRev = 1080; //2*540 based on observed counting on leading and falling edge

// define target angle, user will input
float targetAngle = 0;
// define PID constants
float Kp = 2.0;
float Ki = 0.16;
float Kd = 0.4; 
float integral = 0;
float lastError = 0;
float currentAngle = 0.001; // Angle set by typing in computer. Need to define out of scope
float output = 0; // PID output. Need to define out of scope

// countPulse function. This will trigger this fcn anytime A changes
void countPulse(){
  if (digitalRead(ENCA) == digitalRead(ENCB)){ // confirmed empirically, this direction increases pulseCount
      pulseCount++;
  } else {
    pulseCount--;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // baud rate matches serial monitor of computer

  // set input pins to read from encoder
  pinMode(ENCA,INPUT_PULLUP); //(PULLUP makes sure the value is never noise, just for saftey)
  pinMode(ENCB, INPUT_PULLUP); 

  // set output pins to use to control the motor
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), countPulse, CHANGE); //run countPulse when ENCA changes
  lastPIDTime = millis(); // initialize t = 0
}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){ // only get the number typed into the computer if something was typed, otherwise don't 
    
    float newTarget = Serial.parseFloat();
    while(Serial.available()) Serial.read(); // force to read all characters typed in so that there isn't leftover
  // reset integral and error when new target
    if (newTarget != targetAngle) {
      integral = 0;
      lastError = 0;
    }
    targetAngle = newTarget;
    commandID++;
  }

  unsigned long now = millis();

  if (now - lastPIDTime >= pidInterval){
    float dt = (now - lastPIDTime) / 1000.0; // time elapsed in seconds
    lastPIDTime = now; // reset error variable

    // find the PID output
    currentAngle = (pulseCount / countsPerRev) * 360; // 0 degrees set to wherever the motor starts
    float error = targetAngle - currentAngle; // positive when target is greater than current angle

    // only adjust integral term if not at full speed
    if (output < 255 && output > -255){
      integral += error * dt; // adjust integral term
    }

    float derivative = (error - lastError) / dt; // adjust derivative term
    output = Kp * error + Ki * integral + Kd * derivative;

    output = constrain(output,-255,255); // make sure output lands in between -255, and 255

    // determine the direction of rotation
    if (output > 0){ // CCW Rotation
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);
    } else { // CW Rotation
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);
    }

    // reset error variables
    lastError = error; 
    analogWrite(ENA,abs(output)); //set PWM value based on the output
  }

  // print outputs to serial
  if (now - lastPrintTime >= printInterval){ // delay the printing so it's easier to see
    Serial.print(commandID);
    Serial.print(",");
    Serial.print(now);
    Serial.print(",");
    Serial.print(currentAngle);
    Serial.print(",");
    Serial.println(targetAngle);
    lastPrintTime = now;
  }
}
