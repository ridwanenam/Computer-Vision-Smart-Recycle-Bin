#include <Servo.h>

// ultrasonic
#define TRIG_ORGANIK 2
#define ECHO_ORGANIK 3
#define TRIG_ANORGANIK 4
#define ECHO_ANORGANIK 5
#define TRIG_B3 6
#define ECHO_B3 7

// stepper
#define ENABLE 8
#define DIRECTION 9
#define STEP 10
#define SPEED 2
#define FORWARD HIGH
#define REVERSE LOW

// servo
Servo myservo;

// buzzer
#define buzzer 12

String command = "";

// konstanta stepper
#define STEPS_PER_REV 200
#define DEGREE_STEP (STEPS_PER_REV / 360.0)
#define STEPS_MANUAL_DEGREES (int)(DEGREE_STEP * 10)

// variabel posisi stepper
int current_position = 0;

bool buzzerState = false;
unsigned long previousBuzzerMillis = 0; 
unsigned long buzzerInterval = 0; 
bool buzzerTone = false;

unsigned long previousDistanceMillis = 0;
const unsigned long distanceInterval = 100; 

unsigned long previousSerialMillis = 0;
const unsigned long serialInterval = 10;

int manual_left_degree = 50;
int manual_right_degree = 50;

int auto_organik_degree = 135;
int auto_anorganik_degree = 235;
int auto_b3_degree = 315;


void setup() {
  Serial.begin(9600);

  pinMode(TRIG_ORGANIK, OUTPUT);
  pinMode(ECHO_ORGANIK, INPUT);
  pinMode(TRIG_ANORGANIK, OUTPUT);
  pinMode(ECHO_ANORGANIK, INPUT);
  pinMode(TRIG_B3, OUTPUT);
  pinMode(ECHO_B3, INPUT);

  pinMode(ENABLE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(STEP, OUTPUT);
  digitalWrite(ENABLE, LOW);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  myservo.attach(11);
  myservo.write(10);

  Serial.println("Arduino is ready!!");
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

void rotateStepper(int steps, bool clockwise) {
  digitalWrite(DIRECTION, clockwise ? FORWARD : REVERSE);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP, HIGH);
    delay(SPEED);
    digitalWrite(STEP, LOW);
    delay(SPEED);
  }
}

void left() {
  rotateStepper(STEPS_MANUAL_DEGREES, false);
  current_position -= manual_left_degree;
  if (current_position < 0) {
    current_position += 360;
  }
}

void right() {
  rotateStepper(STEPS_MANUAL_DEGREES, true);
  current_position += manual_right_degree;
  if (current_position >= 360) {
    current_position -= 360;
  }
}

void reset() {
  int steps_to_zero = (int)(current_position * DEGREE_STEP);
  rotateStepper(steps_to_zero, false);
  current_position = 0;
  delay(1000);
  myservo.write(0);
}

void openServo() {
  myservo.write(70);
  delay(2000);
  myservo.write(10);
  delay(1000);
}

void autoOrganik() {
  reset(); 
  int steps_organic_degrees = (int)(auto_organik_degree * DEGREE_STEP); 
  rotateStepper(steps_organic_degrees, true); 
  openServo(); 
  reset(); 
}

void autoAnorganik() {
  reset(); 
  int steps_anorganic_degrees = (int)(auto_anorganik_degree * DEGREE_STEP); 
  rotateStepper(steps_anorganic_degrees, true); 
  openServo(); 
  reset(); 
}

void autoB3() {
  reset(); 
  int steps_b3_degrees = (int)(auto_b3_degree * DEGREE_STEP);
  rotateStepper(steps_b3_degrees, true);
  openServo();
  reset(); 
}

void buzzerOn() {
  buzzerState = true; 
}

void buzzerOff() {
  buzzerState = false;
  digitalWrite(buzzer, LOW); 
}

void handleBuzzer() {
  if (buzzerState) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousBuzzerMillis >= buzzerInterval) {
      previousBuzzerMillis = currentMillis;

      if (buzzerTone) {
        digitalWrite(buzzer, LOW);
        buzzerTone = false;
        buzzerInterval = 1;
      } else {
        digitalWrite(buzzer, HIGH);
        buzzerTone = true;
        buzzerInterval = 2;
      }
    }
  } else {
    digitalWrite(buzzer, LOW);
  }
}

void handleDistance() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousDistanceMillis >= distanceInterval) {
    previousDistanceMillis = currentMillis;

    float distanceOrganik = getDistance(TRIG_ORGANIK, ECHO_ORGANIK);
    float distanceAnorganik = getDistance(TRIG_ANORGANIK, ECHO_ANORGANIK);
    float distanceB3 = getDistance(TRIG_B3, ECHO_B3);

    Serial.print("");
    Serial.print(distanceOrganik);
    Serial.print(",");
    Serial.print(distanceAnorganik);
    Serial.print(",");
    Serial.println(distanceB3);
  }
}

void handleSerial() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("cmd:")) {  
      String cmd = command.substring(4);  
      if (cmd == "Manual") {
        Serial.println("Manual Mode diterima datanya");
      } else if (cmd == "Auto") {
        Serial.println("Auto Mode diterima datanya");
      } else if (cmd == "LEFT") {
        Serial.println("LEFT diterima datanya");
        left();
      } else if (cmd == "RIGHT") {
        Serial.println("RIGHT diterima datanya");
        right();
      } else if (cmd == "OPEN") {
        Serial.println("OPEN diterima datanya");
        openServo();
      } else if (cmd == "RESET") {
        Serial.println("RESET diterima datanya");
        reset();
      } else if (cmd == "AutoOrganik") {
        Serial.println("AutoOrganik diterima datanya");
        autoOrganik();
      } else if (cmd == "AutoAnorganik") {
        Serial.println("AutoAnorganik diterima datanya");
        autoAnorganik();
      } else if (cmd == "AutoB3") {
        Serial.println("AutoB3 diterima datanya");
        autoB3();
      } else if (cmd == "BuzzerON") {
        Serial.println("Buzzer on diterima datanya");
        buzzerOn();
      } else if (cmd == "BuzzerOFF") {
        Serial.println("Buzzer off diterima datanya");
        buzzerOff();
      } else {
        Serial.print("No Command: ");
        Serial.println(cmd);
      }
    }
  }
}

void loop() {
  handleSerial();
  handleDistance();
  handleBuzzer();
}
