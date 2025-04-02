const int MOTOR_A1 = 10;
const int MOTOR_A2 = 11;
const int MOTOR_B1 = 5;
const int MOTOR_B2 = 6;
const int TRIG = 7;
const int ECHO = 9;
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
    digitalWrite(MOTOR_A1, LOW);
}
void loop() {

    moveForward();
    delay(2000);
    moveBackward();
    delay(2000);
    turnRight();
    delay(700);
    moveForward();
    delay(2000);
    moveBackward();
    delay(2000);
    turnLeft();
    delay(1400);
    moveForward();
    delay(2000);
    moveBackward();
    delay(2000);
    stopMotors();
   
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


