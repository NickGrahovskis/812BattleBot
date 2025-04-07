#include <Adafruit_NeoPixel.h>

// Robot states for the main loop
enum RobotState {
  WAITING,
  CALIBRATE,
  GRAB_CONE,
  MAZE,
  FINISH
};

RobotState currentState = WAITING;


// MOTOR PINS AND SPEEDS
#define MOTOR_A1  11  // Left Motor – Backward 
#define MOTOR_A2  10  // Left Motor – Forward
#define MOTOR_B1  9   // Right Motor – Forward 
#define MOTOR_B2  8   // Right Motor – Backward

#define MOTOR_A_SPEED 254  // Motor A speed
#define MOTOR_B_SPEED 254  // Motor B speed


// ULTRASONIC SENSOR PINS AND VARIABLES
#define RIGHT_TRIG  A0
#define RIGHT_ECHO  6
float RIGHT_DIS;

#define FRONT_TRIG  4
#define FRONT_ECHO  5
float FRONT_DIS;

static unsigned long rightUltrasonicTimer = 0;
static unsigned long leftUltrasonicTimer = 0;
static unsigned long frontUltrasonicTimer = 0;

#define LEFT_TRIG  A1
#define LEFT_ECHO  7
float LEFT_DIS;

// ENCODER PINS AND VARIABLES
#define ENCODER_LEFT  3    // Left encoder pin (interrupt pin)
#define ENCODER_RIGHT  2   // Right encoder pin (interrupt pin)
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;


// ROTATION AND ENCODING CONSTANTS
#define ROTATION_SPEED 200
#define ENCODETIME 10

#define TURN_90_LEFT 14
#define TURN_90_RIGHT 9
#define TURN_180 28


// GRIPPER SETTINGS
int OPEN_GRIPPER = 1500;
int CLOSE_GRIPPER = 1000;

// SERVO_INTERVAL
 int SERVO_INTERVAL = 20;

// ROTATION COUNTERS (For rotation based on encoder pulses)

volatile unsigned long RRotation = 0;
volatile unsigned long LRotation = 0;

// PID VARIABLES FOR MOVEMENT
long previousError = 0;
long integral = 0;

// PID constants for motor correction during forward motion
float Kp = 20;
float Ki = 0;  
float Kd = 0;  

// STATEMENTS
bool sensorsCalibrated = false;
bool hasExecuted = false;
bool start = false;
bool mazeLogic = false;
bool onSquare = false;

// TIME VARIABLES
unsigned long mazeStartTime = 0;
const int FORWARD_TILL_CONE = 1200;
const int FORWARD_ENTER_MAZE = 2500;

// LINE SENSOR SETTINGS
#define NUM_SENSOR 6 // Number of sensors
int sensorPins[NUM_SENSOR] = { A2, A3, A4, A5, A6, A7};  // Sensor pin mapping
int sensorValues[NUM_SENSOR];      // Array to store sensor readings
int sensorMin[NUM_SENSOR];
int sensorMax[NUM_SENSOR];
int sensorThreshold[NUM_SENSOR];

// NEOPIXEL SETTINGS
#define LEDPIN 12
int NUMPIXELS = 4;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_RGB + NEO_KHZ800);

// ULTRASONIC DISTANCE VARIABLES AND SAFETY DISTANCES
long duration;
float distance;
#define SAFE_DISTANCE 15
#define STOP_DISTANCE 10


// SETUP FUNCTION
void setup() {
  Serial.begin(9600);
  pixels.begin();
  pixels.show();
  
  // Initialize ultrasonic sensor pins
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  
  // Initialize motor control pins And gripper
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(GRIPPER, OUTPUT);
  
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countPulseLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countPulseRight, CHANGE);

  // Update sensor readings for initial calibration/delay
  updateSensorRight();
  delay(100);
  updateSensorFront();
  delay(100);
  updateSensorLeft();
  delay(100);
}


// MAIN LOOP
void loop() {
  switch (currentState) {
    case WAITING:
      openGripper();
      stop_Pixel();
      canStart();  // Sets 'start' based on the front sensor reading
      if (start) {
        delay(3000);
        currentState = CALIBRATE;
      }
      break;
      
    case CALIBRATE:
      calibrateSensors();
      if(sensorsCalibrated) {
        currentState = GRAB_CONE;
      }
      break;
      
    case GRAB_CONE:
      grabCone();  // Perform the cone grabbing action
      // After grabbing, move left a bit forward to enter the maze
      Rotate_L(TURN_90_LEFT);
      forwardForShortTime(FORWARD_ENTER_MAZE);
      stop_();
      // Record the time when maze navigation begins
      currentState = MAZE;
      break;
      
    case MAZE:
      updateSensorRight();
      updateSensorFront();
      holdingTheCone();
      
      // Normal maze navigation logic based on front sensor readings
      if (FRONT_DIS > 10) {
        normal_Pixel();
        adjustSteering();
      } else {
        updateSensorLeft();
        if (LEFT_DIS < 9) {
          Rotate_L(TURN_180);
          updateSensorFront();
          updateSensorRight();
          if (FRONT_DIS > 10) {
            normal_Pixel();
            adjustSteering();
          }
        } else {
          Rotate_L(TURN_90_LEFT);
        }
      }
      // Transition to FINISH state when a black square is detected by all sensors
      if(isOnBlackSquare()) {
        currentState = FINISH;
      }
      break;
      
    case FINISH:
      // When a black square is detected, the robot stops and opens the gripper to release the cone.
      stop_();
      stop_Pixel();
      openGripper();
      break;
  }
}

// SENSOR AND SENSOR CALIBRATION FUNCTIONS

// Checks if all line sensors read "black" (above the calibrated threshold)
bool isOnBlackSquare() {
  int blackDetected = 0;
  readSensors();
  for (int i = 0; i < NUM_SENSOR; i++) {
    if (sensorValues[i] > sensorThreshold[i]) {
      blackDetected++;
    }
  }
  return blackDetected == NUM_SENSOR;
}

// Reads analog values from line sensors
void readSensors() {
  for (int i = 0; i < NUM_SENSOR; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

// Calibrates line sensors by moving forward and recording min/max values
void calibrateSensors() {
  forwardForShortTime(FORWARD_TILL_CONE);
  if (onSquare) {
    stop_();
    // Calculate and store threshold values based on min and max readings
    for (int i = 0; i < NUM_SENSOR; i++) {
      sensorThreshold[i] = ((sensorMin[i] + sensorMax[i]) / 2) + 50;
    }
    sensorsCalibrated = true;
    return;
  }
}

// MOTOR CONTROL FUNCTIONS

// Moves the robot forward while using encoder feedback for correction
void moveForward() {
  int leftSpeed = MOTOR_A_SPEED;
  int rightSpeed = MOTOR_B_SPEED;
  long error = pulseCountLeft - pulseCountRight;
  integral += error;
  long derivative = error - previousError;
  long correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  leftSpeed -= correction / 2;
  rightSpeed += correction / 2;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, leftSpeed);
  analogWrite(MOTOR_B1, rightSpeed);
  analogWrite(MOTOR_B2, 0);
  rescue();
  previousError = error;
}

// Moves forward for a specific period while updating sensor calibration values
void forwardForShortTime(int timerForward) {
  for (int i = 0; i < NUM_SENSOR; i++) {
    sensorMin[i] = 1023;  // Initialize min to maximum ADC value
    sensorMax[i] = 0;     // Initialize max to minimum ADC value
  }
  unsigned long startingTime = millis();
  while (millis() - startingTime < timerForward) {
    moveForward();
    readSensors();
    for (int i = 0; i < NUM_SENSOR; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      if (sensorValue < sensorMin[i]) {
        sensorMin[i] = sensorValue;
      }
      if (sensorValue > sensorMax[i]) {
        sensorMax[i] = sensorValue;
      }
    }
  }
  onSquare = true;
}

// Stops all motors
void stop_() {
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
}

// Rotates the robot to the right for a given pulse count
void Rotate_R(int Pulses) {
  stop_();  // Ensure the robot is stationary before turning
  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (pulseCountRight < Pulses) {
    // Left wheels move forward
    analogWrite(MOTOR_A2, ROTATION_SPEED);
    analogWrite(MOTOR_A1, 0);
    // Right wheels move backward
    analogWrite(MOTOR_B2, ROTATION_SPEED);
    analogWrite(MOTOR_B1, 0);
    Right_Pixel();
    rescue();  // Safety function to recover from stalls
  }
  stop_();
}

// Rotates the robot to the left for a given pulse count
void Rotate_L(int Pulses) {
  stop_();  // Ensure the robot is stationary before turning
  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();
  while (LRotation < Pulses) {
    // Update left encoder rotation count
    // (updaterotation_LeftEncoder is used here to update LRotation)
    updaterotation_LeftEncoder();
    // Left wheels move backward
    analogWrite(MOTOR_A1, ROTATION_SPEED);
    analogWrite(MOTOR_A2, 0);
    // Right wheels move forward
    analogWrite(MOTOR_B1, ROTATION_SPEED);
    analogWrite(MOTOR_B2, 0);
    Left_Pixel();
    rescue();  // Safety function to recover from stalls
  }
  stop_();
}

// Encoder Interrupts 

// Increments left pulse count on encoder state change
void countPulseLeft() {
  pulseCountLeft++;
}

// Increments right pulse count on encoder state change
void countPulseRight() {
  pulseCountRight++;
}

//updaterotation_RightEncoder
void updaterotation_RightEncoder() {
  static unsigned long timer;
  static bool lastState = false;
  noInterrupts();
  if (millis() > timer) {
    bool state = digitalRead(ENCODER_RIGHT);
    if (lastState != state) {
      RRotation++;
      lastState = state;
    }
    timer = millis() + ENCODETIME;
  }
  interrupts();
}


// Updates left encoder rotation count based on a timer (used in Rotate_L)
void updaterotation_LeftEncoder() {
  static unsigned long timer;
  static bool lastState = false;
  noInterrupts();
  if (millis() > timer) {
    bool state = digitalRead(ENCODER_LEFT);
    if (lastState != state) {
      LRotation++;
      lastState = state;
    }
    timer = millis() + ENCODETIME;
  }
  interrupts();
}

//------------------------------
// ULTRASONIC SENSOR UPDATE FUNCTIONS
//------------------------------

// Update right ultrasonic sensor reading (every ~100ms)
void updateSensorRight() {
  if (millis() > rightUltrasonicTimer) {
    Right_Sensor();
    rightUltrasonicTimer = millis() + 100;
  }
}

// Update left ultrasonic sensor reading (every ~100ms)
void updateSensorLeft() {
  if (millis() > leftUltrasonicTimer) {
    Left_Sensor();
    leftUltrasonicTimer = millis() + 100;
  }
}

// Update front ultrasonic sensor reading (every ~100ms)
void updateSensorFront() {
  if (millis() > frontUltrasonicTimer) {
    Front_Sensor();
    frontUltrasonicTimer = millis() + 100;
  }
}

// Reads and averages the right ultrasonic sensor measurements
void Right_Sensor() {
  float distanceArr[2] = { 0.0, 0.0 };
  float duration;
  for (int i = 0; i < 2; i++) {
    digitalWrite(RIGHT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(RIGHT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(RIGHT_TRIG, LOW);
    duration = pulseIn(RIGHT_ECHO, HIGH);
    distanceArr[i] = (duration * 0.0343) / 2;
  }
  if (distanceArr[0] > 0 && distanceArr[1] > 0 && abs(distanceArr[0] - distanceArr[1]) <= 3) {
    RIGHT_DIS = round((distanceArr[0] + distanceArr[1]) / 2);
  }
}

// Reads and averages the left ultrasonic sensor measurements
void Left_Sensor() {
  float distanceArr[2] = { 0.0, 0.0 };
  float duration;
  for (int i = 0; i < 2; i++) {
    digitalWrite(LEFT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(LEFT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(LEFT_TRIG, LOW);
    duration = pulseIn(LEFT_ECHO, HIGH);
    distanceArr[i] = (duration * 0.0343) / 2;
  }
  if (distanceArr[0] > 0 && distanceArr[1] > 0 && abs(distanceArr[0] - distanceArr[1]) <= 3) {
    LEFT_DIS = round((distanceArr[0] + distanceArr[1]) / 2);
  }
}

// Reads and averages the front ultrasonic sensor measurements
void Front_Sensor() {
  float distanceArr[2] = { 0.0, 0.0 };
  float duration;
  for (int i = 0; i < 2; i++) {
    digitalWrite(FRONT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_TRIG, LOW);
    duration = pulseIn(FRONT_ECHO, HIGH);
    distanceArr[i] = (duration * 0.0343) / 2;
  }
  if (distanceArr[0] > 0 && distanceArr[1] > 0 && abs(distanceArr[0] - distanceArr[1]) <= 3) {
    FRONT_DIS = round((distanceArr[0] + distanceArr[1]) / 2);
    Serial.print("Front sensor reading: ");
    Serial.println(FRONT_DIS);
  }
}

//------------------------------
// GRIPPER FUNCTIONS
//------------------------------

// Grabs the cone by gradually closing the gripper
void grabCone() {
  if (!hasExecuted) {
    for (int i = 0; i < 10; i++) {  // Close gripper gradually
      gripper(CLOSE_GRIPPER);
      delay(20);
    }
    hasExecuted = true;
    mazeLogic = true;
  }
}

// Opens the gripper gradually
void openGripper() {
  for (int i = 0; i < 10; i++) {  // Open gripper with 10 pulses
    gripper(OPEN_GRIPPER);
    delay(20);
  }
}

// Controls the gripper pulse to open/close
void gripper(int pulse) {
  static unsigned long timer;
  static unsigned int last_Pulse;
  if (millis() > timer) {
    if (pulse > 0) {
      last_Pulse = pulse;
    }
    digitalWrite(GRIPPER, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER, LOW);
    timer = millis() + SERVO_INTERVAL;
  }
}

//------------------------------
// START CONDITION FUNCTION
//------------------------------

// Checks the front sensor reading to determine if the robot can start moving
void canStart() {
  updateSensorFront();
  if (FRONT_DIS < 15 && FRONT_DIS > 5) {
    start = true;
  } else {
    start = false;
  }
}

//------------------------------
// NEOPIXEL INDICATOR FUNCTIONS
//------------------------------

// Sets a normal color pattern for the NeoPixels
void normal_Pixel() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(75, 0, 0));
  pixels.setPixelColor(1, pixels.Color(75, 0, 0));
  pixels.setPixelColor(2, pixels.Color(255, 255, 75));
  pixels.setPixelColor(3, pixels.Color(255, 255, 75));
  pixels.show();
}

// Sets a left turn color pattern for the NeoPixels
void Left_Pixel() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255, 43, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(255, 43, 0));
  pixels.show();
}

// Sets a right turn color pattern for the NeoPixels
void Right_Pixel() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(255, 43, 0));
  pixels.setPixelColor(2, pixels.Color(255, 43, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.show();
}

//------------------------------
// UNUSED: back_Pixel
// (This function is defined but not used in the current program logic.)
//------------------------------
/*
void back_Pixel() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.setPixelColor(1, pixels.Color(255, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.show();
}
*/

// Sets a stop color pattern for the NeoPixels
void stop_Pixel() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
}

//------------------------------
// SAFETY AND RESCUE FUNCTIONS
//------------------------------

// Checks for stall conditions and commands a backward move if needed
void rescue() {
  static unsigned long lastRotationTime = millis();
  static int lastRRotation = 0;
  static int lastLRotation = 0;
  
  if (pulseCountLeft != lastLRotation || pulseCountRight != lastRRotation) {
    lastRotationTime = millis();
    lastRRotation = pulseCountRight;
    lastLRotation = pulseCountLeft;
  } else if (millis() - lastRotationTime > 1000) {
    moveBackward(500);
    lastRotationTime = millis();
    return;
  }
}

// Moves the robot backward for a specified time (in milliseconds)
void moveBackward(int time) {
  unsigned long startingTime = millis();
  while (millis() - startingTime < time) {
    analogWrite(MOTOR_A1, MOTOR_A_SPEED);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, MOTOR_B_SPEED);
  }
}

// Maintains the cone grip by repeatedly commanding the gripper to close
void holdingTheCone() {
  for (int i = 0; i < 10; i++) {
    gripper(CLOSE_GRIPPER);
    delay(20);
  }
}

/*
//------------------------------
// UNUSED: calculateSteeringPID
// (A PID function for steering that is defined but not used in the current adjustSteering() implementation.)
//------------------------------
int calculateSteeringPID(int steeringError) {
  // Proportional term
  float proportional = steeringKp * steeringError;
  // Integral term
  steeringIntegral += steeringError;
  float integralTerm = steeringKi * steeringIntegral;
  // Derivative term
  float derivative = steeringError - prevSteeringError;
  float derivativeTerm = steeringKd * derivative;
  // Update previous error
  prevSteeringError = steeringError;
  // Constrain integral term to prevent windup
  steeringIntegral = constrain(steeringIntegral, -50, 50);
  // Return the combined PID output
  return proportional + integralTerm + derivativeTerm;
}
*/

// Calculates a simple steering error based on the right sensor reading (target is 10 cm)
int calculateSteeringError() {
  updateSensorRight();
  int targetDistance = 10; // Target wall distance is 10 cm
  if (RIGHT_DIS > 9 && RIGHT_DIS < 11) {
    return RIGHT_DIS - targetDistance;
  }
  return 0;
}

//------------------------------
// STEERING ADJUSTMENT FUNCTION
//------------------------------

// Adjusts motor speeds to steer the robot while moving forward
void adjustSteering() {
  int steeringError = calculateSteeringError();
  // Instead of using the full PID calculation for steering, a simplified approach is used:
  int steeringPIDOutput = constrain((RIGHT_DIS - 7) * Kp, -125, 125);
  rescue();
  int baseSpeed = 200;  // Base speed for motors
  int leftSpeed = baseSpeed + steeringPIDOutput;
  int rightSpeed = baseSpeed - steeringPIDOutput;
  // Ensure speeds are within valid PWM limits
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  // Apply adjusted motor speeds
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, leftSpeed);
  analogWrite(MOTOR_B1, rightSpeed);
  analogWrite(MOTOR_B2, 0);
}
