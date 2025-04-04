#define MOTOR_A1  11  // Left Motor – Backward 
#define MOTOR_A2  10  // Left Motor – Forward
#define MOTOR_B1  9   // Right Motor – Forward 
#define MOTOR_B2  8   // Right Motor – Backward

#define MOTOR_A_SPEED 235  // Motor A speed
#define MOTOR_B_SPEED 240  // Motor B speed

// ULTRASONIC SENSOR PINS 
#define RIGHT_TRIG  A0
#define RIGHT_ECHO  6
float RIGHT_DIS;

#define FRONT_TRIG  4
#define FRONT_ECHO  5
float FRONT_DIS;
static unsigned long frontUltrasonicTimer = 0;

#define LEFT_TRIG  A1
#define LEFT_ECHO  7
float LEFT_DIS;

#define ENCODER_LEFT  3    // Left encoder pin (interrupt pin)
#define ENCODER_RIGHT  2   // Right encoder pin (interrupt pin)
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;

#define ROTATION_SPEED 200
#define ENCODETIME 10

#define TURN_180 18
#define TURN_90_LEFT 12
#define TURN_90_RIGHT 9
 
int OPEN_GRIPPER = 0;
int CLOSE_GRIPPER = 1000;

int previousTime = 0 ;

int SERVO_INTERVAL = 20;

volatile unsigned long RRotation = 0;
volatile unsigned long LRotation = 0;

long previousError = 0;
long integral = 0;
volatile int pulseCountLeft = 0;
volatile int pulseCountRight = 0;

float Kp = 20;
float Ki = 0;  
float Kd = 0;  

float steeringKp = 1.5;
float steeringKi = 0.1;
float steeringKd = 0.5;
int prevSteeringError = 0;
long steeringIntegral = 0;

#define GRIPPER 13



#define LEDPIN 12
int NUMPIXELS = 4;

long duration;
float distance;

#define SAFE_DISTANCE 15
#define STOP_DISTANCE 10

//Line Sensors
#define NUM_SENSORS 6 // Number of sensors
int sensorPins[NUM_SENSORS] = { A2, A3, A4, A5, A6, A7};  // Sensor pin mapping
int sensorValues[NUM_SENSORS];  // Array to store sensor readings
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS];

bool sensorsCalibrated = false;

bool hasExecuted = false;
bool start = false;
bool Finish = false ;
void setup() {
  Serial.begin(9600);

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(GRIPPER, OUTPUT);
  // digitalWrite(GRIPPER, HIGH);
  

  updateSensorFront();
}

void loop() {
    // updateSensorFront();
    canStart();
    if (start ) {
    grabCone();
    }
  
  Serial.println(FRONT_DIS);
  
}
    
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

  // Serial.print("Left Pulses: ");
  // Serial.print(pulseCountLeft);
  // Serial.print(" | Right Pulses: ");
  // Serial.println(pulseCountRight);

  // Serial.print("leftSpeed: ");
  // Serial.print(leftSpeed);
  // Serial.print(" rightSpeed: ");
  // Serial.println(rightSpeed);

  previousError = error;
}

void forwardTillCone() {
  unsigned long startingTime = millis();
  while (millis() - startingTime < 1000) {
    moveForward();
  }
}

void stop_() {
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
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

void Rotate_L(int Pulses) {
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (LRotation < Pulses) {
    updaterotation_LeftEncoder();

    analogWrite(MOTOR_A1, ROTATION_SPEED);
    analogWrite(MOTOR_A2, 0);

    analogWrite(MOTOR_B1, ROTATION_SPEED);
    analogWrite(MOTOR_B2, 0);

    // Serial.print("LRotation: ");
    // Serial.println(LRotation);
  }
  stop_();
}



void updateSensorFront() {
  if (millis() > frontUltrasonicTimer) {
    Front_Sensor();
    frontUltrasonicTimer = millis() + 100;
  }
}

void Front_Sensor() {
  float distanceArr[2] = {0.0, 0.0};
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

void grabCone()
{
    if (!hasExecuted) // Check if it hasn't executed and flagUp is true
    {
        openGripper();  
        forwardTillCone();// move forward until over square
          for (int i = 0; i < 10; i++) {  // close gripper 10 pulses per 200 mx
            gripper(1000);
            delay(20);
          }
        Rotate_L(TURN_90_LEFT);  // spin onto track
        stop_();
        hasExecuted = true; // Mark as executed
    }
}

void openGripper()
{
  for (int i = 0; i < 10; i++) {  // Open gripper with 10 pulses
        gripper(2500);
        delay(20);
    }
}

void gripper(int pulse) {
  static unsigned long timer;
  static unsigned int   last_Pulse;
  
  if (millis() > timer) {
    // Update LAST_PULSE if a valid pulse is provided; otherwise, reuse the last value.
    if (pulse > 0) {
      last_Pulse = pulse;
    }
    digitalWrite(GRIPPER, HIGH);       // Start pulse
    delayMicroseconds(pulse);          // Hold HIGH for 'pulse' duration
    digitalWrite(GRIPPER, LOW);         // End pulse
    timer = millis() + 20;             // Next pulse after 20ms
  }
}

void canStart()
{
  updateSensorFront();
  if(FRONT_DIS < 15 && FRONT_DIS > 5)
  {
      start = true;
  }
  else
  {
      start = false;
  }

}