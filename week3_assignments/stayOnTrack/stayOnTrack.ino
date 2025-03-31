// Motor control pins
const int MOTOR_A1 = 10;
const int MOTOR_A2 = 11;
const int MOTOR_B1 = 5;
const int MOTOR_B2 = 6;

// Ultrasonic sensor pins
const int TRIG = 7;
const int ECHO = 9;

// Base motor speed
const int baseSpeed = 200;

// Line sensor setup with 6 sensors: using pins A7, A2, A3, A4, A5, A6
// Note: A7 is used in place of the former A1.
const int sensorPins[6] = { A7, A2, A3, A4, A5, A6 };
const int numSensors = 6;
int sensorValues[6];

// Assign weights for proportional control
// Left-most sensor (A7) gets -3, then -2, -1, then +1, +2, +3 for the right-most sensor (A6).
int sensorWeights[6] = { -3, -2, -1, 1, 2, 3 };

// Thresholds for white and black detection (adjust these values based on your sensors and surface)
const int whiteThreshold = 600;
const int blackThreshold = 900;

// Proportional control constant
const float Kp = 30.0;

void setup() {
  // Set motor control pins as outputs
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  // Set ultrasonic sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  Serial.begin(9600);
}

// Function to measure distance using the ultrasonic sensor
long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Set motor speeds (allowing negative values for reverse)
// Positive values drive forward; negative values drive reverse.
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor control
  if (leftSpeed >= 0) {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, leftSpeed);
  } else {
    analogWrite(MOTOR_A1, -leftSpeed);
    analogWrite(MOTOR_A2, 0);
  }
  
  // Right motor control
  if (rightSpeed >= 0) {
    analogWrite(MOTOR_B1, rightSpeed);
    analogWrite(MOTOR_B2, 0);
  } else {
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, -rightSpeed);
  }
}

// Stop all motors
void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void loop() {
  // Check for obstacles using the ultrasonic sensor
  long distance = getDistance();
  if (distance > 0 && distance < 20) {
    stopMotors();
    Serial.println("Obstacle detected! Stopping...");
    delay(100);
    return;  // Skip further processing while an obstacle is present
  }
  
  int weightedSum = 0;
  int activeSensors = 0;
  
  // Read each sensor value and compute a weighted sum based on line detection
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    Serial.print(sensorValues[i]);
    Serial.print("\t");
    
    // Consider the sensor active if it detects the black line
    if (sensorValues[i] > blackThreshold) {
      weightedSum += sensorWeights[i];
      activeSensors++;
    }
  }
  Serial.println();

  // Recovery routine: if no sensor detects the line
  if (activeSensors == 0) {
    Serial.println("Line lost! Reversing and pivoting...");
    // Reverse briefly to try to re-acquire the line
    setMotorSpeeds(-baseSpeed, -baseSpeed);
    delay(200);
    // Pivot turn (for example, pivot left) to search for the line
    setMotorSpeeds(-baseSpeed, baseSpeed);
    delay(150);
    return;  // Skip the rest of this loop cycle
  }

  // Normal line following using proportional control
  float error = (float)weightedSum / activeSensors;
  int correction = Kp * error;
  int leftMotorSpeed = baseSpeed - correction;
  int rightMotorSpeed = baseSpeed + correction;
  
  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
  
  // Debugging output to the Serial Monitor
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print("  Correction: ");
  Serial.print(correction);
  Serial.print("  Left Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("  Right Speed: ");
  Serial.println(rightMotorSpeed);
  
  delay(50);
}
