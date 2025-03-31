#define GRIPPER_OPEN 1500  // Pulse width for open position
#define GRIPPER_CLOSE 1000 // Pulse width for closed position
const int SERVO = 13;
const int MOTOR_A1 = 10;
const int MOTOR_A2 = 11;
const int MOTOR_B1 = 5;
const int MOTOR_B2 = 6;
const int TRIG = 7;
const int ECHO = 9;
const int MOTOR_A_SPEED = 250;
const int MOTOR_B_SPEED = 237;
const int ROTATION_SPEED = 200;
const int CLOSE_GRIPPER = 1000; // For clarity, same as GRIPPER_CLOSE
const int OPEN_GRIPPER = 1500;  // For clarity, same as GRIPPER_OPEN

// ------------------------------
// Setup Function
// ------------------------------
void setup() {
  Serial.begin(9600);
  
  // Setup gripper servo pin
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);
  
  // Setup ultrasonic sensor pins
  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);
  pinMode(ECHO, INPUT);
  
  // Setup motor pins
  pinMode(MOTOR_A1, OUTPUT);
  digitalWrite(MOTOR_A1, LOW);
  pinMode(MOTOR_A2, OUTPUT);
  digitalWrite(MOTOR_A2, LOW);
  pinMode(MOTOR_B1, OUTPUT);
  digitalWrite(MOTOR_B1, LOW);
  pinMode(MOTOR_B2, OUTPUT);
  digitalWrite(MOTOR_B2, LOW);
  
  // Gripper Pattern Sequence at Startup:
  // 1. Open gripper for 1 second
  // 2. Close gripper for 1 second
  // 3. Open gripper for 1 second
  unsigned long start = millis();
  while (millis() - start < 1000) {
    gripper(OPEN_GRIPPER);
  }
  
  start = millis();
  while (millis() - start < 1000) {
    gripper(CLOSE_GRIPPER);
  }
  
  start = millis();
  while (millis() - start < 1000) {
    gripper(OPEN_GRIPPER);
  }
}

// ------------------------------
// Main Loop: Movement & Gripper Sequence
// ------------------------------
void loop() {
  // First 25cm: Move forward with gripper open
  moveForward();
  delay(1300);
  stopMotors();
  delay(500);
  
  // Prepare to grab: Close gripper for 1 second
  unsigned long start = millis();
  while (millis() - start < 1000) {
    gripper(CLOSE_GRIPPER);
  }
  
  // Second 25cm: Move forward while keeping gripper closed continuously
  start = millis();
  while (millis() - start < 1300) {
    moveForward();
    gripper(CLOSE_GRIPPER);  // Keep sending closed pulses to hold the grip
  }
  stopMotors();
  
  // End of sequence: Stop the robot
  while (true) {
    stopMotors();
    delay(1000);
  }
}

// ------------------------------
// Function: Send Servo Pulse to Gripper
// ------------------------------
void gripper(int pulse) {
  static unsigned long timer;
  static unsigned int LAST_PULSE;
  
  if (millis() > timer) {
    // Update LAST_PULSE if a valid pulse is provided; otherwise, reuse the last value.
    if (pulse > 0) {
      LAST_PULSE = pulse;
    }
    else {
      pulse = LAST_PULSE;
    }
    
    digitalWrite(SERVO, HIGH);       // Start pulse
    delayMicroseconds(pulse);          // Hold HIGH for 'pulse' duration
    digitalWrite(SERVO, LOW);         // End pulse
    timer = millis() + 20;             // Next pulse after 20ms
  }
}

// ------------------------------
// Function: Move Forward
// ------------------------------
void moveForward() {
  // Left motor: A2 on, A1 off
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, MOTOR_A_SPEED);
  
  // Right motor: B1 on, B2 off
  analogWrite(MOTOR_B1, MOTOR_B_SPEED);
  analogWrite(MOTOR_B2, 0);
}

// ------------------------------
// Function: Move Backward
// ------------------------------
void moveBackward() {
  // Left motor: A1 on, A2 off
  analogWrite(MOTOR_A1, MOTOR_A_SPEED);
  analogWrite(MOTOR_A2, 0);
  
  // Right motor: B2 on, B1 off
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, MOTOR_B_SPEED);
}

// ------------------------------
// Function: Stop Motors
// ------------------------------
void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

// ------------------------------
// Function: Turn Left
// ------------------------------
void turnLeft() {
  analogWrite(MOTOR_A1, ROTATION_SPEED);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, ROTATION_SPEED);
  analogWrite(MOTOR_B2, 0);
}

// ------------------------------
// Function: Turn Right
// ------------------------------
void turnRight() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, ROTATION_SPEED);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, ROTATION_SPEED);
}

// ------------------------------
// Function: Get Distance from Ultrasonic Sensor
// ------------------------------
long getDistance() {
  digitalWrite(TRIG, LOW);   // Clear trigger pin
  delayMicroseconds(2);
  
  digitalWrite(TRIG, HIGH);  // Send 10us pulse
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH); // Measure echo pulse width
  long distance = duration * 0.034 / 2;   // Convert to cm
  
  return distance;
}
