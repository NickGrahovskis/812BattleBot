// #define MOTOR_A1  11  // Left Motor – Backward 
// #define MOTOR_A2  10  // Left Motor – Forward
// #define MOTOR_B1  9   // Right Motor – Forward 
// #define MOTOR_B2  8   // Right Motor – Backward

// #define MOTOR_A_SPEED 235  // Motor A speed
// #define MOTOR_B_SPEED 240  // Motor B speed

// //  ULTRASONIC SENSOR PINS 
// // Right sensor (for wall following)
// #define RIGHT_TRIG  A0
// #define RIGHT_ECHO  6
// float RIGHT_DIS;

// // Front sensor (for obstacle detection)
// #define FRONT_TRIG  4
// #define FRONT_ECHO  5
// float FRONT_DIS;

// // Left sensor (for obstacle detection)
// #define LEFT_TRIG  A1
// #define LEFT_ECHO  7
// float LEFT_DIS;
// //  ENCODER / ROTATION CONFIGURATION 
// #define ENCODER_LEFT  3    // Left encoder pin (interrupt pin)
// #define ENCODER_RIGHT  2   // Right encoder pin (interrupt pin)
// volatile unsigned long leftPulses = 0;
// volatile unsigned long rightPulses = 0;
// // Rotation parameters
// #define ROTATION_SPEED 200
// #define ENCODETIME 10

// #define TURN_180 20
// #define TURN_90_LEFT 7
// #define TURN_90_RIGHT 9

// volatile unsigned long RRotation = 0;
// volatile unsigned long LRotation = 0;

// long previousError = 0;
// long integral = 0;
// volatile int pulseCountLeft = 0;
// volatile int pulseCountRight = 0;

// float Kp = 7.25;    
// float Ki = 1.25;  
// float Kd = 0.75;  

// #define PIVALUE  3.141592653589793238462643383279502884197
// #define GRIPPER 12
// #define LEDPIN 13

// long duration;
// float distance;

// #define SAFE_DISTANCE 10
// #define STOP_DISTANCE 7

// const int SIDE_WALL[] = {6,11};


// const int Num_Led = 4;

// void setup() {
//   Serial.begin(9600);
//   pinMode(FRONT_TRIG, OUTPUT);
//   pinMode(FRONT_ECHO, INPUT);
//   pinMode(LEFT_TRIG, OUTPUT);
//   pinMode(LEFT_ECHO, INPUT);
//   pinMode(RIGHT_TRIG, OUTPUT);
//   pinMode(RIGHT_ECHO, INPUT);
//   pinMode(MOTOR_A1, OUTPUT);
//   pinMode(MOTOR_A2, OUTPUT);
//   pinMode(MOTOR_B1, OUTPUT);
//   pinMode(MOTOR_B2, OUTPUT);

//   attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countPulseLeft, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countPulseRight, CHANGE);
// }

// void loop()
//  {
//    if(RIGHT_DIS < SIDE_WALL[1] && RIGHT_DIS > SIDE_WALL[0])
//    {
//      if(FRONT_DIS >= STOP_DISTANCE && FRONT_DIS <= SAFE_DISTANCE)
//      {
//       moveForward();
//      }
//      else if (FRONT_DIS <= STOP_DISTANCE)
//      {
//       if(LEFT_DIS <= SIDE_WALL[0])
//       {
//           Rotate_R(TURN_180);
//       }
//       else 
//       {
//         Rotate_L(TURN_90_LEFT);
//       }
//      }
//    }
//    else
//    {
//       Forward_BEFORE_RIGHT();
//       Rotate_R(TURN_90_RIGHT);
//    }

// }
// void countPulseLeft() {
//   pulseCountLeft++;
// }

// void countPulseRight() {
//   pulseCountRight++;
// }

// void resetPulseCount() {
//   pulseCountLeft = 0;
//   pulseCountRight = 0;
// }

// void moveForward() {
  
//   int leftSpeed = MOTOR_A_SPEED;
//   int rightSpeed = MOTOR_B_SPEED;

//   // Calculate the error (difference in pulse counts between the motors)
//   long error = pulseCountLeft - pulseCountRight;
  
//   // Calculate the integral term (sum of all previous errors)
//   integral += error;
  
//   long derivative = error - previousError;
  
//   long correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
//   leftSpeed -= correction / 2;  // Scale down correction
//   rightSpeed += correction / 2; // Scale down correction


//   if (leftSpeed < 0) leftSpeed = 0;
//   if (rightSpeed < 0) rightSpeed = 0;
//   if (leftSpeed > 255) leftSpeed = 255;
//   if (rightSpeed > 255) rightSpeed = 255;
 
//   analogWrite(MOTOR_A1, 0);         
//   analogWrite(MOTOR_A2, leftSpeed);
//   analogWrite(MOTOR_B1, rightSpeed); 
//   analogWrite(MOTOR_B2, 0);         

//   Serial.print("Left Pulses: ");
//   Serial.print(pulseCountLeft);
//   Serial.print(" | Right Pulses: ");
//   Serial.println(pulseCountRight);

//   Serial.print("leftSpeeed ");
//   Serial.print(leftSpeed);
//    Serial.print("rightSpeed ");
//   Serial.print(rightSpeed);

//   // Update previous error for the next iteration
//   previousError = error;
// }
// void updaterotation_RightEncoder() // Right encoder update
// {
//   static unsigned long timer;
//   static bool lastState = false;
//   noInterrupts();
//   if (millis() > timer) {
//     bool state = digitalRead(ENCODER_RIGHT);
//     if (lastState != state) {
//       RRotation++;
//       lastState = state;
//     }
//     timer = millis() + ENCODETIME;
//   }
//   interrupts();
// }

// void updaterotation_LeftEncoder() // Left encoder update
// {
//   static unsigned long timer;
//   static bool lastState = false;
//   noInterrupts();
//   if (millis() > timer) {
//     bool state = digitalRead(ENCODER_LEFT);
//     if (lastState != state) {
//       LRotation++;
//       lastState = state;
//     }
//     timer = millis() + ENCODETIME;
//   }
//   interrupts();
// }

// void Rotate_R(int Pulses) 
// {
//   stop_();  // Ensure the robot is stationary before turning

//   noInterrupts();  // Prevent any sensor update interruptions
//   RRotation = 0;
//   LRotation = 0;
//   interrupts();

//   while (RRotation < Pulses) {  // Rotate until the right encoder count reaches threshold
//     // Left wheels move forward
//     analogWrite(MOTOR_A2, ROTATION_SPEED);  // Left forward at defined speed
//     analogWrite(MOTOR_A1, 0);               // Left backward OFF

//     // Right wheels move backward
//     analogWrite(MOTOR_B2, ROTATION_SPEED);  // Right backward at defined speed
//     analogWrite(MOTOR_B1, 0);               // Right forward OFF
    
//     Serial.print(RRotation);
//     failSafe(); // Call safety function (implement as needed)
//   }
//   stop_();  // Stop after rotation
// }

// void Rotate_L(int Pulses) {
//   stop_();  // Ensure the robot is stationary before turning

//   noInterrupts();
//   RRotation = 0;
//   LRotation = 0;
//   interrupts();

//   while (LRotation <= Pulses) {  // Rotate until the left encoder count reaches threshold
//     // Left wheels move backward
//     analogWrite(MOTOR_A1, ROTATION_SPEED);  // Left backward at defined speed
//     analogWrite(MOTOR_A2, 0);               // Left forward OFF

//     // Right wheels move forward
//     analogWrite(MOTOR_B1, ROTATION_SPEED);  // Right forward at defined speed
//     analogWrite(MOTOR_B2, 0);               // Right backward OFF
//   }
//   stop_();  // Stop after rotation
// }

// void stop_() {
//   analogWrite(MOTOR_A2, 0);
//   analogWrite(MOTOR_B2, 0);
//   analogWrite(MOTOR_A1, 0);
//   analogWrite(MOTOR_B1, 0);
// }

//   void updaterotation_RightEncoder();
//   void updaterotation_LeftEncoder();
//   void Rotate_L();
//   void Rotate_R();

// void Initializing() {
//   // Dummy function – add your initialization code here if needed
// }

// void failSafe() {
//   // Dummy function – add your safety code here if needed
// }

// void updateSensorRight() { // Right sonar update
//   static unsigned long timer = 0;
//   if (millis() > timer) {
//     Right_Sensor();
//     timer = millis() + 150; // update every 150ms (adjust as needed)
//   }
// }

// void updateSensorLeft() { // Left sonar update
//   static unsigned long timer = 0;
//   if (millis() > timer) {
//     Left_Sensor();
//     timer = millis() + 150;
//   }
// }

// void updateSensorFront() { // Front sonar update
//   static unsigned long timer = 0;
//   if (millis() > timer) {
//     Front_Sensor();
//     timer = millis() + 150;
//   }
// }

// // ----- Sensor Reading Functions -----
// void Right_Sensor() // right sonar
// {
//   float distance[2] = {0.0, 0.0};
//   float duration;
//     for(int i = 0; i < 2; i++)
//     {
//     digitalWrite(RIGHT_TRIG, LOW);
//     delayMicroseconds(2);
//     digitalWrite(RIGHT_TRIG, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(RIGHT_TRIG, LOW);

//     duration = pulseIn(RIGHT_ECHO, HIGH);
//     distance[i] = (duration*.0343)/2;
//     }
//     if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3)
//     {
//       RIGHT_DIS = round((distance[1] + distance[0]) / 2);
//     }
// }

// void Left_Sensor() // right sonar
// {
//   float distance[2] = {0.0, 0.0};
//   float duration;
//     for(int i = 0; i < 2; i++)
//     {
//     digitalWrite(LEFT_TRIG, LOW);
//     delayMicroseconds(2);
//     digitalWrite(LEFT_TRIG, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(LEFT_TRIG, LOW);

//     duration = pulseIn(LEFT_ECHO, HIGH);
//     distance[i] = (duration*.0343)/2;
//     }
//     if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3)
//     {
//       LEFT_DIS = round((distance[1] + distance[0]) / 2);
//     }
// }

// void Front_Sensor() // front sonar
// {
//   float distance[2] = {0.0, 0.0};
//   float duration;
//     for(int i = 0; i < 2; i++)
//     {
//     digitalWrite(FRONT_TRIG, LOW);
//     delayMicroseconds(2);
//     digitalWrite(FRONT_TRIG, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(FRONT_TRIG, LOW);

//     duration = pulseIn(FRONT_ECHO, HIGH);
//     distance[i] = (duration*.0343)/2;
//     }
//     if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3)
//     {
//       FRONT_DIS = round((distance[1] + distance[0]) / 2);
//     }
// }
// void Call_UltraSonic_Sensors()
// {
//   Right_Sensor();
//   Left_Sensor();
//   Front_Sensor();
// }

// void Update_UltraSonic_Sensors()
// {
//   updateSensorFront()
//   updateSensorLeft()
//   updateSensorRight()
// }

// void Update_Rotation_Sensors()
// {
//      updaterotation_RightEncoder();
//       updaterotation_LeftEncoder();
// }

// void Forward_BEFORE_RIGHT()
// {
//  unsigned long startTime = millis();
//   while (millis() - startTime < 2000) {
//     moveForward();
//   }
// }

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
// Changed variable name from FORWARD_DIS to FRONT_DIS to match later usage.
#define FRONT_TRIG  4
#define FRONT_ECHO  5
float FRONT_DIS;

// Left sensor (for obstacle detection)
#define LEFT_TRIG  A1
#define LEFT_ECHO  7
float LEFT_DIS;

//  ENCODER / ROTATION CONFIGURATION 
#define ENCODER_LEFT  3    // Left encoder pin (interrupt pin)
#define ENCODER_RIGHT  2   // Right encoder pin (interrupt pin)
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
// Rotation parameters
#define ROTATION_SPEED 200
#define ENCODETIME 10

#define TURN_180 20
#define TURN_90_LEFT 7
#define TURN_90_RIGHT 9

volatile unsigned long RRotation = 0;
volatile unsigned long LRotation = 0;

unsigned long resetTimer = 0;
const unsigned long backupDuration = 5000;

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

long duration;
float distance;

#define SAFE_DISTANCE 10
#define STOP_DISTANCE 7

const int SIDE_WALL[] = {6,11};

const int Num_Led = 4;

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
}

void loop() {
  // Update sensor values at the beginning of the loop
  updateSensorRight();
  updateSensorFront();
  updateSensorLeft();

  Backup_function();

  // Decision logic based on sensor readings
  if (RIGHT_DIS < SIDE_WALL[1] && RIGHT_DIS > SIDE_WALL[0]) {
    if (FRONT_DIS >= STOP_DISTANCE && FRONT_DIS <= SAFE_DISTANCE) {
      moveForward();
    } else if (FRONT_DIS < STOP_DISTANCE) {
      if (LEFT_DIS <= SIDE_WALL[0]) {
        Rotate_R(TURN_180);
      } else {
        Rotate_L(TURN_90_LEFT);
      }
    }
  } else {
    Forward_BEFORE_RIGHT();
    Rotate_R(TURN_90_RIGHT);
  }
  Backup_function();
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

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation < Pulses) {
    // Update right encoder rotation count
    updaterotation_RightEncoder();

    // Left wheels move forward
    analogWrite(MOTOR_A2, ROTATION_SPEED);
    analogWrite(MOTOR_A1, 0);

    // Right wheels move backward
    analogWrite(MOTOR_B2, ROTATION_SPEED);
    analogWrite(MOTOR_B1, 0);
    
    Serial.print("RRotation: ");
    Serial.println(RRotation);
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
  static unsigned long timer = 0;
  if (millis() > timer) {
    Right_Sensor();
    timer = millis() + 150; // Update every 150ms
  }
}

void updateSensorLeft() { // Left sonar update
  static unsigned long timer = 0;
  if (millis() > timer) {
    Left_Sensor();
    timer = millis() + 150;
  }
}

void updateSensorFront() { // Front sonar update
  static unsigned long timer = 0;
  if (millis() > timer) {
    Front_Sensor();
    timer = millis() + 150;
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
    // Fixed array indexing: average the two readings using distance[0] and distance[1]
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

void moveBackward(){

    analogWrite(MOTOR_A1, MOTOR_A_SPEED);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, MOTOR_B_SPEED);
}

void Backup_function() {
    if (RRotation == 0 || LRotation == 0) {  
        if (millis() - resetTimer >= backupDuration) { 
            moveBackward();
            resetTimer = millis();  // Reset timer
        }
    }

    
}





