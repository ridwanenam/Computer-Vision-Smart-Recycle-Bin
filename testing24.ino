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
#define SPEED 3
#define FORWARD HIGH
#define REVERSE LOW

// servo
Servo myservo;

// buzzer
#define buzzer 12
bool buzzerState = false;
unsigned long previousBuzzerMillis = 0; 
unsigned long buzzerInterval = 0; 
bool buzzerTone = false;

// konstanta stepper
#define STEPS_PER_REV 200
#define DEGREE_STEP (STEPS_PER_REV / 360.0)
#define STEPS_MANUAL_DEGREES (int)(DEGREE_STEP * 10)

#define QUEUE_SIZE 10

// variabel posisi stepper
int current_position = 0;

int manual_left_degree = 1;
int manual_right_degree = 1;

// queue serial command
String commandQueue[QUEUE_SIZE];
int queueStart = 0;
int queueEnd = 0;

bool isQueueFull() {
  return ((queueEnd + 1) % QUEUE_SIZE == queueStart);
}

bool isQueueEmpty() {
  return (queueStart == queueEnd);
}

bool enqueue(String command) {
  if (isQueueFull()) {
    Serial.println("Queue fulled!");
    return false;
  }
  commandQueue[queueEnd] = command;
  queueEnd = (queueEnd + 1) % QUEUE_SIZE;
  return true;
}

String dequeue() {
  if (isQueueEmpty()) {
    return "";
  }
  String command = commandQueue[queueStart];
  queueStart = (queueStart + 1) % QUEUE_SIZE;
  return command;
}

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
  rotateStepper(STEPS_MANUAL_DEGREES * 20, false);
  current_position -= manual_left_degree;
  if (current_position < 0) {
    current_position += 360;
  }
}

void right() {
  rotateStepper(STEPS_MANUAL_DEGREES * 20, true);
  current_position += manual_right_degree;
  if (current_position >= 360) {
    current_position -= 360;
  }
}

void openServo() {
  myservo.write(70);
  delay(1250);
  myservo.write(10);
}

void reset() {
  if (current_position != 0) {
    int steps_cw = current_position * DEGREE_STEP;
    int steps_ccw = (360 - current_position) * DEGREE_STEP;

    if (steps_cw <= steps_ccw) {
      rotateStepper(steps_cw, false); // CounterClockWise
    } else {
      rotateStepper(steps_ccw, true); // Clockwise
    }
  }

  current_position = 0;
  delay(1000);
  myservo.write(10);
}

void moveToDegree(int target_degree) {
  int target_steps = (int)(target_degree * DEGREE_STEP); 
  int steps_to_move;

  if (target_degree >= current_position) {
    steps_to_move = target_steps - (int)(current_position * DEGREE_STEP);
    rotateStepper(steps_to_move, true); // ClockWise
  } else {
    steps_to_move = (360 - current_position + target_degree) * DEGREE_STEP;
    rotateStepper(steps_to_move, true); 
  }
  current_position = target_degree; 
}

void autoOrganik() {
  moveToDegree(100); 
  delay(1000);
  openServo(); 
  reset(); 
}

void autoAnorganik() {
  moveToDegree(430); 
  delay(1000);
  openServo(); 
  reset(); 
}

void autoB3() {
  moveToDegree(300); 
  delay(1000);
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
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 750) {
    lastMillis = currentMillis;

    float distanceOrganik = getDistance(TRIG_ORGANIK, ECHO_ORGANIK);
    float distanceAnorganik = getDistance(TRIG_ANORGANIK, ECHO_ANORGANIK);
    float distanceB3 = getDistance(TRIG_B3, ECHO_B3);

    Serial.print(distanceOrganik);
    Serial.print(",");
    Serial.print(distanceAnorganik);
    Serial.print(",");
    Serial.println(distanceB3);
  }
}

void handleSerial() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("cmd:")) {
      String cmd = command.substring(4);
      if (!enqueue(cmd)) {
        Serial.println("Queue full, can't handle command anymore.");
      }
    }
  }
}

void processQueue() {
  if (!isQueueEmpty()) {
    String command = dequeue();
    if (command == "LEFT") {
      left();
      Serial.println("LEFT process.");
    } else if (command == "RIGHT") {
      right();
      Serial.println("RIGHT process.");
    } else if (command == "OPEN") {
      openServo();
      Serial.println("OPEN process.");
    } else if (command == "RESET") {
      reset();
      Serial.println("RESET process.");
    } else if (command == "AutoOrganik") {
      autoOrganik();
      Serial.println("AutoOrganik process.");
    } else if (command == "AutoAnorganik") {
      autoAnorganik();
      Serial.println("AutoAnorganik process.");
    } else if (command == "AutoB3") {
      autoB3();
      Serial.println("AutoB3 process.");
    } else if (command == "BuzzerON") {
      buzzerOn();
      Serial.println("BuzzerON process.");
    } else if (command == "BuzzerOFF") {
      buzzerOff();
      Serial.println("BuzzerOFF process.");
    } else if (command == "Auto") {;
      Serial.println("AUTO MODE.");
    } else if (command == "Manual") {
      Serial.println("MANUAL MODE.");
    } else {
      Serial.print("No Command: ");
      Serial.println(command);
    }
  }
}

void loop() {
  handleSerial();
  processQueue();
  handleDistance();
  handleBuzzer();
}
