#include <Adafruit_NeoPixel.h>
#define LED_PIN    13      
#define NUM_LEDS   4

#define MOTOR_A1  11  // Left Motor – Backward 
#define MOTOR_A2  10  // Left Motor – Forward
#define MOTOR_B1  9   // Right Motor – Forward 
#define MOTOR_B2  8   // Right Motor – Backward

#define MOTOR_A_SPEED 235  // Motor A speed
#define MOTOR_B_SPEED 240  // Motor B speed

//  ULTRASONIC SENSOR PINS 
// Right sensor (for wall following)
#define RIGHT_TRIG  A0
#define RIGHT_ECHO  6
float RIGHT_DIS;

// Front sensor (for obstacle detection)
#define FRONT_TRIG  4
#define FRONT_ECHO  5
float FRONT_DIS;

// Left sensor (for obstacle detection)
#define LEFT_TRIG  A1
#define LEFT_ECHO  7
float LEFT_DIS;

// Ultrasonic sensor timers
static unsigned long rightUltrasonicTimer = 0;
static unsigned long leftUltrasonicTimer = 0;
static unsigned long frontUltrasonicTimer = 0;

//  ENCODER / ROTATION CONFIGURATION 
#define ENCODER_LEFT  3    // Left encoder pin (interrupt pin)
#define ENCODER_RIGHT  2   // Right encoder pin (interrupt pin)
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
// Rotation parameters
#define ROTATION_SPEED 200
#define ENCODETIME 10

#define TURN_180 15
#define TURN_90_LEFT 12// 12
#define TURN_90_RIGHT 9

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile unsigned long RRotation = 0;
volatile unsigned long LRotation = 0;

long previousError = 0;
long integral = 0;
volatile int pulseCountLeft = 0;
volatile int pulseCountRight = 0;

float Kp = 15;    //15
float Ki = 0;  
float Kd = 0;  

// PID Steering Variables
float steeringKp = 1.5;    // Proportional gain for steering
float steeringKi = 0.1;    // Integral gain for steering
float steeringKd = 0.5;    // Derivative gain for steering
int prevSteeringError = 0;
long steeringIntegral = 0;

#define PIVALUE  3.141592653589793238462643383279502884197
#define GRIPPER 12
#define LEDPIN 13

long duration;
float distance;

#define SAFE_DISTANCE 15
#define STOP_DISTANCE 10 // 10

const int SIDE_WALL[] = {6.5,11}; // 5 11

const int Num_Led = 4;

// New PID Steering Function
int calculateSteeringError() {
  updateSensorRight();
  
  // Target wall distance is 10 cm
  int targetDistance = 10;
  
  // If right distance is between 9 and 11 cm, calculate steering error
  if (RIGHT_DIS > 9 && RIGHT_DIS < 11) {
    return RIGHT_DIS - targetDistance;
  }
  
  // If outside the range, return 0 (no correction needed)
  return 0;
}

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

void adjustSteering() {
  int steeringError = calculateSteeringError();
  int steeringPIDOutput = constrain((RIGHT_DIS - 7) * Kp , -125 , 125); //calculateSteeringPID(steeringError);

  // Base speed for motors
  int baseSpeed = 200;

  // Adjust motor speeds based on steering error
  int leftSpeed = baseSpeed + steeringPIDOutput;
  int rightSpeed = baseSpeed - steeringPIDOutput;

  // Constrain speeds to valid PWM range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Apply motor speeds
  analogWrite(MOTOR_A1, 0);         
  analogWrite(MOTOR_A2, leftSpeed);
  analogWrite(MOTOR_B1, rightSpeed); 
  analogWrite(MOTOR_B2, 0);  
}

void setup() {

  Serial.begin(9600);
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countPulseLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countPulseRight, CHANGE);

  updateSensorRight();
  delay(100);
  updateSensorFront();
  delay(100);
  updateSensorLeft();
  delay(100);
}

void loop() {
  updateSensorRight();
  updateSensorFront();

  // Update sensor values at the beginning of the loop
 
  // Decision logic based on sensor readings
  //if (RIGHT_DIS > SIDE_WALL[0]) {
    if (FRONT_DIS > STOP_DISTANCE) {
      // Adjust steering while moving forward
      adjustSteering();
    } else /*if (FRONT_DIS < STOP_DISTANCE)*/ {
      updateSensorLeft();
      if (LEFT_DIS < SIDE_WALL[0]) {
        DeadEndFunction();
        // Rotate_L(TURN_180);
      } else {
        Rotate_L(TURN_90_LEFT);
      }
    }
  // } else {
  //   Forward_BEFORE_RIGHT();
  //   Rotate_R(TURN_90_RIGHT);
  // }
}

void countPulseLeft() {
  pulseCountLeft++;
}

void countPulseRight() {
  pulseCountRight++;
}

void resetPulseCount() {
  pulseCountLeft = 0;
  pulseCountRight = 0;
}

void moveForward() {
  int leftSpeed = MOTOR_A_SPEED;
  int rightSpeed = MOTOR_B_SPEED;

  // Calculate the error (difference in pulse counts between the motors)
  long error = pulseCountLeft - pulseCountRight;
  
  // Calculate the integral term (sum of all previous errors)
  integral += error;
  
  long derivative = error - previousError;
  
  long correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  leftSpeed -= correction / 2;  // Scale down correction
  rightSpeed += correction / 2; // Scale down correction

  if (leftSpeed < 0) leftSpeed = 0;
  if (rightSpeed < 0) rightSpeed = 0;
  if (leftSpeed > 255) leftSpeed = 255;
  if (rightSpeed > 255) rightSpeed = 255;
 
  analogWrite(MOTOR_A1, 0);         
  analogWrite(MOTOR_A2, leftSpeed);
  analogWrite(MOTOR_B1, rightSpeed); 
  analogWrite(MOTOR_B2, 0);         

  Serial.print("Left Pulses: ");
  Serial.print(pulseCountLeft);
  Serial.print(" | Right Pulses: ");
  Serial.println(pulseCountRight);

  Serial.print("leftSpeed: ");
  Serial.print(leftSpeed);
  Serial.print(" rightSpeed: ");
  Serial.println(rightSpeed);

  // Update previous error for the next iteration
  previousError = error;
}

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

void Rotate_R(int Pulses) {
  stop_();  // Ensure the robot is stationary before turning

  // noInterrupts();
  // RRotation = 0;
  // LRotation = 0;
  // interrupts();
  pulseCountLeft = 0;
  while (pulseCountRight < Pulses) {
    // Update right encoder rotation count
    // updaterotation_RightEncoder();

    // Left wheels move forward
    analogWrite(MOTOR_A2, ROTATION_SPEED);
    analogWrite(MOTOR_A1, 0);

    // Right wheels move backward
    analogWrite(MOTOR_B2, ROTATION_SPEED);
    analogWrite(MOTOR_B1, 0);
    
    // Serial.print("RRotation: ");
    // Serial.println(RRotation);
    failSafe(); // Safety function (implement as needed)
  }
  stop_();
}

void Rotate_L(int Pulses) {
  stop_();  // Ensure the robot is stationary before turning

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (LRotation < Pulses) {
    // Update left encoder rotation count
    updaterotation_LeftEncoder();

    // Left wheels move backward
    analogWrite(MOTOR_A1, ROTATION_SPEED);
    analogWrite(MOTOR_A2, 0);

    // Right wheels move forward
    analogWrite(MOTOR_B1, ROTATION_SPEED);
    analogWrite(MOTOR_B2, 0);
    
    Serial.print("LRotation: ");
    Serial.println(LRotation);
    failSafe(); // Safety function (implement as needed)

  }
  stop_();
}

void stop_() {
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
}

void Initializing() {
  // Dummy function – add your initialization code here if needed
}

void failSafe() {
  // Dummy function – add your safety code here if needed
}

void updateSensorRight() { // Right sonar update
  if (millis() > rightUltrasonicTimer) {
    Right_Sensor();
    rightUltrasonicTimer = millis() + 100; // Update every 150ms
  }
}

void updateSensorLeft() { // Left sonar update
  if (millis() > leftUltrasonicTimer) {
    Left_Sensor();
    leftUltrasonicTimer = millis() + 100;
  }
}

void updateSensorFront() { // Front sonar update
  if (millis() > frontUltrasonicTimer) {
    Front_Sensor();
    frontUltrasonicTimer = millis() + 100;
  }
}

// ----- Sensor Reading Functions -----
void Right_Sensor() {
  float distance[2] = {0.0, 0.0};
  float duration;
  for (int i = 0; i < 2; i++) {
    digitalWrite(RIGHT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(RIGHT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(RIGHT_TRIG, LOW);

    duration = pulseIn(RIGHT_ECHO, HIGH);
    distance[i] = (duration * 0.0343) / 2;
  }
  if (distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3) {
    RIGHT_DIS = round((distance[0] + distance[1]) / 2);
  }
}

void Left_Sensor() {
  float distance[2] = {0.0, 0.0};
  float duration;
  for (int i = 0; i < 2; i++) {
    digitalWrite(LEFT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(LEFT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(LEFT_TRIG, LOW);

    duration = pulseIn(LEFT_ECHO, HIGH);
    distance[i] = (duration * 0.0343) / 2;
  }
  if (distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3) {
    LEFT_DIS = round((distance[0] + distance[1]) / 2);
  }
}

void Front_Sensor() {
  float distance[2] = {0.0, 0.0};
  float duration;
  for (int i = 0; i < 2; i++) {
    digitalWrite(FRONT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_TRIG, LOW);

    duration = pulseIn(FRONT_ECHO, HIGH);
    distance[i] = (duration * 0.0343) / 2;
  }
  if (distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3) {
    FRONT_DIS = round((distance[0] + distance[1]) / 2);
  }
}

void Forward_BEFORE_RIGHT() {
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    moveForward();
  }
}

void moveBackward()
{
  analogWrite(MOTOR_A1, MOTOR_A_SPEED);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, MOTOR_B_SPEED);
}

void moveBackwardForShortTime(){
  unsigned long startingTime = millis();
  while (millis() - startingTime < 1000) {
    moveBackward();
  }
}

void DeadEndFunction() {
  unsigned long startingTime = millis();
  while (millis() - startingTime < 1000) {

  }
  moveBackwardForShortTime();
  Rotate_L(TURN_180);
}

void stuckFunction(){
   unsigned long startingTime = millis();
   if(LRotation < 5 || RRotation < 5){
    if(millis() - startingTime >= 5000)
    {
     moveBackwardForShortTime();
      startingTime = 0;
    }
   }
}

//not completely working 