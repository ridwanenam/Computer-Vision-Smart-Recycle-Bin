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
