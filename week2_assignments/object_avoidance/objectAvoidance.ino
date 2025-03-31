
const int MOTOR_A1 = 10;
const int MOTOR_A2 = 11;
const int MOTOR_B1 = 5;
const int MOTOR_B2 = 6;

const int TRIG_LEFT = 7;
const int TRIG_RIGHT = 13;
const int TRIG = 7;

const int ECHO = 9;
const int ECHO_LEFT = 5;
const int ECHO_RIGHT = 3;
const int MOTOR_A_SPEED = 250;
const int MOTOR_B_SPEED = 237;
const int ROTATION_SPEED = 200;

void setup() {
  
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);
}
void loop() {
      
    Serial.print(distance);
    // Debug: print the distance value to monitor what's happening
    

      if (distance > 10) {  // Move forward if no obstacle is within 5 cm
          moveForward();
      } else {
          stopMotors();
          delay(500);
          turnRight();
          delay(700);
          moveForward();
          delay(1000);
          turnLeft();
          delay(700);
          moveForward();
          delay(1500);
          turnLeft();
          delay(700);
          moveForward();
          delay(1000);
          turnRight();
          delay(700);
      }
}
long getDistance() {
    digitalWrite(TRIG, LOW);  // Clear the trigger pin
    delayMicroseconds(2);
    
    digitalWrite(TRIG, HIGH); // Send a 10us pulse
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);  // Stop pulse
    
    long duration = pulseIn(ECHO, HIGH); // Measure echo time
    long distance = duration * 0.034 / 2;   // Convert to cm
    
    
    return distance; // Return distance value
}

// Motor control functions
void moveForward() {
    analogWrite(MOTOR_A1, 0);  // Left motor forward (A2 ON, A1 OFF)
    analogWrite(MOTOR_A2, MOTOR_A_SPEED);
    analogWrite(MOTOR_B1, MOTOR_B_SPEED); // Right motor forward (B1 ON, B2 OFF)
    analogWrite(MOTOR_B2, 0);
}

void moveBackward() {
    analogWrite(MOTOR_A1, MOTOR_A_SPEED); // Left motor backward (A1 ON, A2 OFF)
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);   // Right motor backward (B2 ON, B1 OFF)
    analogWrite(MOTOR_B2, MOTOR_B_SPEED);
}

void stopMotors() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 0);
}


void turnLeft() {
    analogWrite(MOTOR_A1, ROTATION_SPEED);
    analogWrite(MOTOR_A2, 0);  // Left motor backward
    analogWrite(MOTOR_B1, ROTATION_SPEED);
    analogWrite(MOTOR_B2, 0);    // Right motor forward
}

void turnRight() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, ROTATION_SPEED);    // Left motor forward
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, ROTATION_SPEED);  // Right motor backward
}


