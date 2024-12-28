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
#define SPEED 3
#define FORWARD HIGH
#define REVERSE LOW

// servo
Servo myservo;

// buzzer
#define BUZZER 12

String command = "";

// konstanta stepper
#define STEPS_PER_REV 200
#define DEGREE_STEP (STEPS_PER_REV / 360.0)
#define STEPS_MANUAL_DEGREES (int)(DEGREE_STEP * 10)

// variabel posisi stepper
int current_position = 0;

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

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  myservo.attach(11);

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
  current_position -= 10;
  if (current_position < 0) {
    current_position += 360;
  }
}

void right() {
  rotateStepper(STEPS_MANUAL_DEGREES, true);
  current_position += 10;
  if (current_position >= 360) {
    current_position -= 360;
  }
}

void reset() {
  int steps_to_zero = (int)(current_position * DEGREE_STEP);
  rotateStepper(steps_to_zero, false);
  current_position = 0;
}

void openServo() {
  myservo.write(90);
  delay(2000);
  myservo.write(0);
}

void autoOrganik() {
  reset(); 
  int steps_organic_degrees = (int)(130 * DEGREE_STEP); 
  rotateStepper(steps_organic_degrees, true); 
  openServo(); 
  reset(); 
}

void autoAnorganik() {
  reset(); 
  int steps_anorganic_degrees = (int)(235 * DEGREE_STEP); 
  rotateStepper(steps_anorganic_degrees, true); 
  openServo(); 
  reset(); 
}

void autoB3() {
  reset(); 
  int steps_b3_degrees = (int)(315 * DEGREE_STEP);
  rotateStepper(steps_b3_degrees, true);
  openServo();
  reset(); 
}


void buzzerOn() {
  digitalWrite(BUZZER, HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER, LOW);
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "Manual") {
      Serial.println("Manual Mode diterima datanya");
    } else if (command == "Auto") {
      Serial.println("Auto Mode diterima datanya");
    } else if (command == "LEFT") {
      Serial.println("LEFT diterima datanya");
      left();
    } else if (command == "RIGHT") {
      Serial.println("RIGHT diterima datanya");
      right();
    } else if (command == "OPEN") {
      Serial.println("OPEN diterima datanya");
      openServo();
    } else if (command == "RESET") {
      Serial.println("RESET diterima datanya");
      reset();
    } else if (command == "AutoOrganik") {
      Serial.println("AutoOrganik diterima datanya");
      autoOrganik();
    } else if (command == "AutoAnorganik") {
      Serial.println("AutoAnorganik diterima datanya");
      autoAnorganik();
    } else if (command == "AutoB3") {
      Serial.println("AutoB3 diterima datanya");
      autoB3();
    } else if (command == "BuzzerON") {
      Serial.println("BuzzerON diterima datanya");
      buzzerOn();
    } else if (command == "BuzzerOFF") {
      Serial.println("BuzzerOFF diterima datanya");
      buzzerOff();
    } else {
      Serial.print("No Command: ");
      Serial.println(command);
    }
  }

  float distanceOrganik = getDistance(TRIG_ORGANIK, ECHO_ORGANIK);
  float distanceAnorganik = getDistance(TRIG_ANORGANIK, ECHO_ANORGANIK);
  float distanceB3 = getDistance(TRIG_B3, ECHO_B3);

  Serial.print(distanceOrganik);
  Serial.print(",");
  Serial.print(distanceAnorganik);
  Serial.print(",");
  Serial.println(distanceB3);
  delay(500);
}
