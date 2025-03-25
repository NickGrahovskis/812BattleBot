
// ----- Sensor Pin Definitions -----
#define RIGHT_TRIG  A0
#define RIGHT_ECHO  6
float RIGHT_DIS = 0;

#define FRONT_TRIG  4
#define FRONT_ECHO  5
float FRONT_DIS = 0;

#define LEFT_TRIG   A1
#define LEFT_ECHO   7
float LEFT_DIS = 0;

// Temporary variables for sensor calculations
long duration;
float distance;

// ----- Sensor Update Functions -----
void updateSensorRight() { // Right sonar update
  static unsigned long timer = 0;
  if (millis() > timer) {
    Right_Sensor();
    timer = millis() + 150; // update every 150ms (adjust as needed)
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
void Right_Sensor() // right sonar
{
  float distance[2] = {0.0, 0.0};
  float duration;
    for(int i = 0; i < 2; i++)
    {
    digitalWrite(RIGHT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(RIGHT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(RIGHT_TRIG, LOW);

    duration = pulseIn(RIGHT_ECHO, HIGH);
    distance[i] = (duration*.0343)/2;
    }
    if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3)
    {
      RIGHT_DIS = round((distance[1] + distance[2]) / 2);
    }
}

void Left_Sensor() // right sonar
{
  float distance[2] = {0.0, 0.0};
  float duration;
    for(int i = 0; i < 2; i++)
    {
    digitalWrite(LEFT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(LEFT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(LEFT_TRIG, LOW);

    duration = pulseIn(LEFT_ECHO, HIGH);
    distance[i] = (duration*.0343)/2;
    }
    if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3)
    {
      LEFT_DIS = round((distance[1] + distance[2]) / 2);
    }
}

void Front_Sensor() // front sonar
{
  float distance[2] = {0.0, 0.0};
  float duration;
    for(int i = 0; i < 2; i++)
    {
    digitalWrite(FRONT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_TRIG, LOW);

    duration = pulseIn(FRONT_ECHO, HIGH);
    distance[i] = (duration*.0343)/2;
    }
    if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 3)
    {
      FRONT_DIS = round((distance[1] + distance[2]) / 2);
    }
}

// ----- Setup and Loop -----
void setup() {
  Serial.begin(9600);
  
  // Set sensor pins
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
}

void loop() {
  // Update sensors
  updateSensorRight();
  updateSensorLeft();
  updateSensorFront();
  
  // Print sensor readings to serial
  Serial.print("Right: ");
  Serial.print(RIGHT_DIS);
  Serial.print(" cm, Front: ");
  Serial.print(FRONT_DIS);
  Serial.print(" cm, Left: ");
  Serial.print(LEFT_DIS);
  Serial.println(" cm");
  
  delay(100); // adjust delay as needed
}






