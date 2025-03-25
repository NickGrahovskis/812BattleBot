// Motor Definitions
#define MOTOR_A1  11 // Left Motor – Backward 
#define MOTOR_A2  10 // Left Motor – Forward
#define MOTOR_B1  9  // Right Motor – Forward 
#define MOTOR_B2  8  // Right Motor – Backward


// 40 pulses is a full turn 

// Ultrasonic Sensor Pins 
// Right sensor (for wall following)
#define RIGHT_TRIG  A0
#define RIGHT_ECHO  6
float RIGHT_DIS;

// Front sensor (for obstacle detection)
#define FRONT_TRIG  4
#define FRONT_ECHO  5
float FORWARD_DIS;

// Left sensor (for obstacle detection)
#define LEFT_TRIG  A1
#define LEFT_ECHO  7
float LEFT_DIS;

// Encoder / Rotation Configuration 
#define ENCODER_LEFT  3    // Left encoder pin (interrupt pin)
#define ENCODER_RIGHT 2    // Right encoder pin (interrupt pin)
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;

// Rotation parameters
#define ROTATION_SPEED 200
#define ENCODETIME 10


// Remove these problematic macros:
// #define RROTATION
// #define LROTATION

// Global rotation counters (used in encoder interrupt routines)
volatile unsigned long RRotation = 0;
volatile unsigned long LRotation = 0;

// Function prototypes
void Initializing();
void failSafe();
void updaterotation_RightEncoder();
void updaterotation_LeftEncoder();
void Rotate_R();
void Rotate_L();
void stop_();


void setup() {
  Serial.begin(9600);
  Initializing();
  
  // Set motor pins as outputs
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  // Set encoder pins as input with pullup
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), updaterotation_LeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), updaterotation_RightEncoder, CHANGE);
}

void loop() {
  // Print sensor readings to serial
  Serial.print("Right: ");
  Serial.print(RRotation);
  Serial.print("  Left: ");
  Serial.println(LRotation);
  
   updaterotation_RightEncoder();
   updaterotation_LeftEncoder();
   Rotate_L();
   Rotate_R();

}

void updaterotation_RightEncoder() // Right encoder update
{
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

void updaterotation_LeftEncoder() // Left encoder update
{
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

void Rotate_R() 
{
  stop_();  // Ensure the robot is stationary before turning

  noInterrupts();  // Prevent any sensor update interruptions
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation < 14) {  // Rotate until the right encoder count reaches threshold
    // Left wheels move forward
    analogWrite(MOTOR_A2, ROTATION_SPEED);  // Left forward at defined speed
    analogWrite(MOTOR_A1, 0);               // Left backward OFF

    // Right wheels move backward
    analogWrite(MOTOR_B2, ROTATION_SPEED);  // Right backward at defined speed
    analogWrite(MOTOR_B1, 0);               // Right forward OFF
    
    Serial.print(RRotation);
    failSafe(); // Call safety function (implement as needed)
  }
  stop_();  // Stop after rotation
}

void Rotate_L() {
  stop_();  // Ensure the robot is stationary before turning

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (LRotation <= 7) {  // Rotate until the left encoder count reaches threshold
    // Left wheels move backward
    analogWrite(MOTOR_A1, ROTATION_SPEED);  // Left backward at defined speed
    analogWrite(MOTOR_A2, 0);               // Left forward OFF

    // Right wheels move forward
    analogWrite(MOTOR_B1, ROTATION_SPEED);  // Right forward at defined speed
    analogWrite(MOTOR_B2, 0);               // Right backward OFF
  }
  stop_();  // Stop after rotation
}

void stop_() {
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
}

  void updaterotation_RightEncoder();
  void updaterotation_LeftEncoder();
  void Rotate_L();
  void Rotate_R();

void Initializing() {
  // Dummy function – add your initialization code here if needed
}

void failSafe() {
  // Dummy function – add your safety code here if needed
}