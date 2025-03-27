#define MOTOR_A1  11  // Left Motor – Backward 
#define MOTOR_A2  10  // Left Motor – Forward
#define MOTOR_B1  9   // Right Motor – Forward 
#define MOTOR_B2  8   // Right Motor – Backward

#define MOTOR_A_SPEED 235  // Motor A speed
#define MOTOR_B_SPEED 240  // Motor B speed

//  ULTRASONIC SENSOR PINS 
// Right sensor (for wall following)
#define RIGHT_TRIG_PIN  A0
#define RIGHT_ECHO_PIN  6
float RIGHT_DIS;
// Front sensor (for obstacle detection)
#define FRONT_TRIG_PIN  4
#define FRONT_ECHO_PIN  5
float FORWARD_DIS;
// Left sensor (for obstacle detection)
#define LEFT_TRIG_PIN  A1
#define LEFT_ECHO_PIN  7
float LEFT_DIS;
//  ENCODER / ROTATION CONFIGURATION 
#define ENCODER_LEFT  3    // Left encoder pin (interrupt pin)
#define ENCODER_RIGHT  2   // Right encoder pin (interrupt pin)
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;

long previousError = 0;
long integral = 0;
volatile int pulseCountLeft = 0;
volatile int pulseCountRight = 0;

float Kp = 7.25;    
float Ki = 1.25;  
float Kd = 0.75;  

#define PIVALUE  3.141592653589793238462643383279502884197
#define GRIPPER 12
#define LEDPIN 13

const int Num_Led = 4;

void setup() {
  Serial.begin(9600);
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countPulseLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countPulseRight, CHANGE);
}

void loop() {
  moveForward();
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

  Serial.print("leftSpeeed ");
  Serial.print(leftSpeed);
   Serial.print("rightSpeed ");
  Serial.print(rightSpeed);

  // Update previous error for the next iteration
  previousError = error;
}


