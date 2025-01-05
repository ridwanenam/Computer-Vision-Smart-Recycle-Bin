#include <Servo.h>

// ultrasonic
#define TRIG_ORGANIK 4
#define ECHO_ORGANIK 5
#define TRIG_ANORGANIK 2
#define ECHO_ANORGANIK 3
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

// stepper constant
#define STEPS_PER_REV 200
#define DEGREE_STEP (STEPS_PER_REV / 360.0)
#define STEPS_MANUAL_DEGREES (int)(DEGREE_STEP * 10)

// stepper position
int current_position = 0;

unsigned long startDetectionTime = 0; 
bool isDetecting = false;             

int manual_left_degree = 1;
int manual_right_degree = 1;


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
  rotateStepper(STEPS_MANUAL_DEGREES*10, false);
  current_position -= manual_left_degree*10;
  if (current_position < 0) {
    current_position += 360;
  }
}

void right() {
  rotateStepper(STEPS_MANUAL_DEGREES*10, true);
  current_position += manual_right_degree*10;
  if (current_position >= 360) {
    current_position -= 360;
  }
}

void openServo() {
  myservo.write(70);
  delay(2000);
  myservo.write(10);
  delay(1000);
}

void moveToDegree(int target_degree) {
  int target_steps = (int)(target_degree * DEGREE_STEP); 
  int steps_to_move;

  if (target_degree >= current_position) {
    steps_to_move = target_steps - (int)(current_position * DEGREE_STEP);
    rotateStepper(steps_to_move, true); 
  } else {
    steps_to_move = (int)((current_position - target_degree) * DEGREE_STEP);
    rotateStepper(steps_to_move, false); 
  }

  current_position = target_degree; 
}


void reset() {
  int steps_to_zero = abs((int)(current_position * DEGREE_STEP));
  
  if (current_position > 0) {
    rotateStepper(steps_to_zero, false); 
  } else if (current_position < 0) {
    rotateStepper(steps_to_zero, true); 
  }
  current_position = 0;
}


void autoOrganik() {
  moveToDegree(-190); 
  delay(1000);
  openServo(); 
  reset(); 
}

void autoAnorganik() {
  moveToDegree(345); 
  delay(1000);
  openServo(); 
  reset();
}

void autoB3() {
  moveToDegree(190); 
  delay(1000);
  openServo(); 
  reset(); 
}

void handleBuzzer() {
    float distanceOrganik = getDistance(TRIG_ORGANIK, ECHO_ORGANIK);
    float distanceAnorganik = getDistance(TRIG_ANORGANIK, ECHO_ANORGANIK);
    float distanceB3 = getDistance(TRIG_B3, ECHO_B3);

    bool isBelowThreshold = (distanceOrganik > 0 && distanceOrganik < 10) || 
                            (distanceAnorganik > 0 && distanceAnorganik < 10) || 
                            (distanceB3 > 0 && distanceB3 < 10);

    if (isBelowThreshold) {
        if (!isDetecting) {
            startDetectionTime = millis();
            isDetecting = true;
        } else if (millis() - startDetectionTime >= 2000) {
            unsigned char i;
            for (i = 0; i < 80; i++) {
                digitalWrite(buzzer, HIGH); // high frequency 
                delayMicroseconds(500);    
                digitalWrite(buzzer, LOW);
                delayMicroseconds(500);
            }
            for (i = 0; i < 100; i++) {
                digitalWrite(buzzer, HIGH); // low frequency
                delayMicroseconds(1000);  
                digitalWrite(buzzer, LOW);
                delayMicroseconds(1000);
            }
        }
    } else {
        isDetecting = false;
        startDetectionTime = 0;
        digitalWrite(buzzer, LOW);
    }
}

void buzzerOff(){
  digitalWrite(buzzer, LOW);
}

void handleDistance() {
    float distanceOrganik = getDistance(TRIG_ORGANIK, ECHO_ORGANIK);
    float distanceAnorganik = getDistance(TRIG_ANORGANIK, ECHO_ANORGANIK);
    float distanceB3 = getDistance(TRIG_B3, ECHO_B3);

    Serial.print("");
    Serial.print(distanceOrganik);
    Serial.print(",");
    Serial.print(distanceAnorganik);
    Serial.print(",");
    Serial.println(distanceB3);
    delay(750);
}

void handleSerial() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("cmd:")) {  
      String cmd = command.substring(4);  
      if (cmd == "Manual") {
        Serial.println("Manual Mode");
      } else if (cmd == "Auto") {
        Serial.println("Auto Mode");
      } else if (cmd == "LEFT") {
        Serial.println("LEFT");
        left();
      } else if (cmd == "RIGHT") {
        Serial.println("RIGHT");
        right();
      } else if (cmd == "OPEN") {
        Serial.println("OPEN");
        openServo();
      } else if (cmd == "RESET") {
        Serial.println("RESET");
        reset();
      } else if (cmd == "AutoOrganik") {
        Serial.println("AutoOrganik");
        autoOrganik();
      } else if (cmd == "AutoAnorganik") {
        Serial.println("AutoAnorganik");
        autoAnorganik();
      } else if (cmd == "AutoB3") {
        Serial.println("AutoB3");
        autoB3();
      } else if (cmd == "BuzzerOFF") {
        Serial.println("Buzzer off");
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
