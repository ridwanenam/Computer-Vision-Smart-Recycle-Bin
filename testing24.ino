/*Pin di arduino udah fix kayanya*/

// Ultrasonic sensor
#define TRIG_ORGANIK 2
#define ECHO_ORGANIK 3
#define TRIG_ANORGANIK 4
#define ECHO_ANORGANIK 5
#define TRIG_B3 6
#define ECHO_B3 7

// Stepper
#define ENABLE 8
#define DIRECTION 9
#define STEP 10
#define FORWARD HIGH
#define REVERSE LOW
/* Change this values to alter the clock speed (speed stepper) */ 
#define SPEED 1

// Servo PWM
#define Servo 11

// Buzzer
#define Buzzer 12


String command = "";

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_ORGANIK, OUTPUT);
  pinMode(ECHO_ORGANIK, INPUT);
  pinMode(TRIG_ANORGANIK, OUTPUT);
  pinMode(ECHO_ANORGANIK, INPUT);
  pinMode(TRIG_B3, OUTPUT);
  pinMode(ECHO_B3, INPUT);

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
      ManualFunction();
    } else if (command == "LEFT") {
      Serial.println("LEFT diterima datanya");
    } else if (command == "RIGHT") {
      Serial.println("RIGHT diterima datanya");
    } else if (command == "OPEN") {
      Serial.println("OPEN diterima datanya");
    } else if (command == "RESET") {
      Serial.println("RESET diterima datanya");
    } else if (command == "Auto") {
      Serial.println("Auto diterima datanya");
    } else if (command == "AutoOrganik") {
      Serial.println("AutoOrganik diterima datanya");
    } else if (command == "AutoAnorganik") {
      Serial.println("AutoAnorganik diterima datanya");
    } else if (command == "AutoB3") {
      Serial.println("AutoB3 diterima datanya");
    } else if (command == "BuzzerON") {
      Serial.println("BuzzerON diterima datanya");
    } else if (command == "BuzzerOFF") {
      Serial.println("BuzzerOFF diterima datanya");
    } else {
      Serial.print("Command tidak ada ");
      Serial.println(command);
    }
  }

  float distanceOrganik = getDistance(TRIG_ORGANIK, ECHO_ORGANIK);
  float distanceAnorganik = getDistance(TRIG_ANORGANIK, ECHO_ANORGANIK);
  float distanceB3 = getDistance(TRIG_B3, ECHO_B3);

  // data distance format JSON
  Serial.print("{\"organik\":");
  Serial.print(distanceOrganik);
  Serial.print(",\"anorganik\":");
  Serial.print(distanceAnorganik);
  Serial.print(",\"b3\":");
  Serial.print(distanceB3);
  Serial.println("}");
  delay(500);
}

void ManualFunction() {
  Serial.println("Entered Manual Mode");
}

